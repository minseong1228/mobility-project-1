#include <iostream>
#include <unordered_map>
#include <vector>
#include <string>
#include <queue>
#include <algorithm>
#include <iomanip>
#include <chrono>
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "tinyxml2.h"

using namespace std;
using namespace tinyxml2;

/* ===================== 상수 ===================== */
const double AVG_SPEED = 13.9; // m/s
static constexpr double R = 6371000.0;

/* ===================== 구조체 ===================== */
struct Node {
    string id;
    double lat = 0.0;
    double lon = 0.0;
};

struct EdgeInfo {
    double length = 0.0;
    string roadName;
};

/* ===================== 전역 ===================== */
unordered_map<string, Node> nodes;
unordered_map<string, vector<pair<string, double>>> adj;
unordered_map<string, unordered_map<string, EdgeInfo>> edgeInfoMap;
unordered_map<string, unordered_map<string, double>> trafficDelay;
unordered_map<string, string> keyIdToName;

/* ===================== 유틸 ===================== */
double haversine(double lat1, double lon1, double lat2, double lon2) {
    auto rad = [](double d) { return d * M_PI / 180.0; };
    double dlat = rad(lat2 - lat1), dlon = rad(lon2 - lon1);
    double a = sin(dlat / 2) * sin(dlat / 2)
        + cos(rad(lat1)) * cos(rad(lat2))
        * sin(dlon / 2) * sin(dlon / 2);
    return 2 * R * atan2(sqrt(a), sqrt(1 - a));
}

string localName(const char* name) {
    if (!name) return "";
    string s = name;
    size_t p = s.find_last_of(":}");
    return (p != string::npos) ? s.substr(p + 1) : s;
}

/* ===================== GraphML 로드 ===================== */
bool loadGraphML(const string& file) {
    XMLDocument doc;
    if (doc.LoadFile(file.c_str()) != XML_SUCCESS) return false;
    XMLElement* root = doc.RootElement();
    if (!root) return false;

    for (auto* key = root->FirstChildElement(); key; key = key->NextSiblingElement()) {
        if (localName(key->Name()) != "key") continue;
        const char* id = key->Attribute("id");
        const char* attr = key->Attribute("attr.name");
        if (id && attr) keyIdToName[id] = attr;
    }

    XMLElement* graph = root->FirstChildElement("graph");
    if (!graph) graph = root;

    for (auto* n = graph->FirstChildElement(); n; n = n->NextSiblingElement()) {
        if (localName(n->Name()) != "node") continue;
        const char* id = n->Attribute("id");
        if (!id) continue;

        Node nd; nd.id = id;
        for (auto* d = n->FirstChildElement(); d; d = d->NextSiblingElement()) {
            if (localName(d->Name()) != "data") continue;
            const char* key = d->Attribute("key");
            const char* text = d->GetText();
            if (!key || !text) continue;

            string attr = keyIdToName[key];
            double v = atof(text);
            if (attr == "lat" || attr == "y" || key == string("d4"))
                nd.lat = v;
            else if (attr == "lon" || attr == "x" || key == string("d5"))
                nd.lon = v;
        }
        nodes[id] = nd;
        adj[id];
    }

    for (auto* e = graph->FirstChildElement(); e; e = e->NextSiblingElement()) {
        if (localName(e->Name()) != "edge") continue;
        const char* s = e->Attribute("source");
        const char* t = e->Attribute("target");
        if (!s || !t) continue;

        double length = haversine(nodes[s].lat, nodes[s].lon,
            nodes[t].lat, nodes[t].lon);

        adj[s].push_back({ t, length });
        edgeInfoMap[s][t] = { length, "" };
        adj[t].push_back({ s, length });
        edgeInfoMap[t][s] = { length, "" };
    }
    return true;
}

/* ===================== Dijkstra (시간 기반) ===================== */
pair<vector<string>, double> dijkstra(const string& start, const string& goal) {
    unordered_map<string, double> dist;
    unordered_map<string, string> prev;

    for (auto& kv : nodes) dist[kv.first] = 1e18;
    dist[start] = 0;

    priority_queue<pair<double, string>,
        vector<pair<double, string>>, greater<>> pq;
    pq.push({ 0, start });

    while (!pq.empty()) {
        auto [cd, u] = pq.top(); pq.pop();
        if (u == goal) break;

        for (auto& pr : adj[u]) {
            string v = pr.first;
            double w = pr.second;

            double travelTime = w / AVG_SPEED;
            double lightDelay = 0.0;

            if (trafficDelay.count(u) && trafficDelay[u].count(v)) {
                lightDelay = trafficDelay[u][v];
            }

            double cost = cd + travelTime + lightDelay;

            if (dist[v] > cost) {
                dist[v] = cost;
                prev[v] = u;
                pq.push({ dist[v], v });
            }
        }
    }

    if (dist[goal] >= 1e18) return { {}, -1 };

    vector<string> path;
    for (string cur = goal; ; cur = prev[cur]) {
        path.push_back(cur);
        if (cur == start) break;
    }
    reverse(path.begin(), path.end());
    return { path, dist[goal] };
}

/* ===================== 출력용 ===================== */
string toDash(const vector<string>& p) {
    string s;
    for (size_t i = 0; i < p.size(); i++) {
        s += p[i];
        if (i + 1 < p.size()) s += "-";
    }
    return s;
}

/* ===================== main ===================== */
int main() {
    if (!loadGraphML("jongro.graphml")) {
        cout << "Graph load failed\n";
        return 0;
    }

    string s, d;
    cout << "Start node id: ";
    cin >> s;
    cout << "Destination node id: ";
    cin >> d;

    int n;
    cout << "신호등 개수 입력: ";
    cin >> n;

    for (int i = 0; i < n; i++) {
        string from, to;
        double delay;
        cout << "from to delay(sec): ";
        cin >> from >> to >> delay;
        trafficDelay[from][to] = delay;
    }

    auto [path, totalTime] = dijkstra(s, d);

    if (path.empty()) {
        cout << "경로 없음\n";
        return 0;
    }

    cout << fixed << setprecision(3);
    cout << "[Dijkstra] Total travel time (sec): " << totalTime << "\n";
    cout << "[Dijkstra] Vehicle route: " << toDash(path) << "\n";

    return 0;
}
