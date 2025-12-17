/*
 Project: Smart Mobility Shortest Path Example
 Description:
  - GraphML 기반 도로 네트워크를 그래프로 구성
  - Dijkstra 알고리즘을 이용해 차량의 최단 경로 탐색
  - Random Path Sampling(Monte Carlo)과 성능 비교

 Author: Minseong
*/

#include <iostream>
#include <unordered_map>
#include <vector>
#include <string>
#include <queue>
#include <limits>
#include <random>
#include <algorithm>
#include <iomanip>
#include <chrono>
#include <cmath>
map<string, double> trafficLightDelay;
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#include "tinyxml2.h"

using namespace tinyxml2;
using namespace std;

/* ================================
   Node : 지도 상의 그래프 노드
   - id : 노드 번호
   - lat/lon : 위도/경도
   ================================ */
struct Node {
    string id;
    double lat = 0.0;
    double lon = 0.0;
};

/* ================================
   EdgeInfo : 간선 정보
   - length : 거리 (미터)
   - roadName : 도로명
   ================================ */
struct EdgeInfo {
    double length = 0.0;
    string roadName;
};

/* ================================
   전역 그래프 구조
   ================================ */
unordered_map<string, Node> nodes;                               // 노드 정보
unordered_map<string, vector<pair<string, double>>> adj;          // 인접리스트: id → (이웃노드, 길이)
unordered_map<string, unordered_map<string, EdgeInfo>> edgeInfoMap; // 상세 간선 정보
unordered_map<string, string> keyIdToName;                        // GraphML key id → field name (d4 = lat 등)

static constexpr double R = 6371000.0; // 지구 반지름(미터)

/* =========================================================
   Haversine 거리 계산 (lat/lon → meter)
   ========================================================= */
double haversine(double lat1, double lon1, double lat2, double lon2) {
    auto rad = [](double d) { return d * M_PI / 180.0; };
    double dlat = rad(lat2 - lat1), dlon = rad(lon2 - lon1);
    double a = sin(dlat / 2) * sin(dlat / 2)
        + cos(rad(lat1)) * cos(rad(lat2)) * sin(dlon / 2) * sin(dlon / 2);
    return 2 * R * atan2(sqrt(a), sqrt(1 - a));
}

/* =========================================================
   XML 태그 이름에서 namespace 제거 (node, edge 등만 추출)
   ========================================================= */
string localName(const char* name) {
    if (!name) return "";
    string s = name;
    size_t p = s.find_last_of(":}");
    return (p != string::npos) ? s.substr(p + 1) : s;
}

/* =========================================================
   GraphML 파일 로드
   - d4 = latitude
   - d5 = longitude
   - d16 = length
   - d13 = road name
   ========================================================= */
bool loadGraphML(const string& file) {
    XMLDocument doc;
    if (doc.LoadFile(file.c_str()) != XML_SUCCESS) return false;
    XMLElement* root = doc.RootElement();
    if (!root) return false;

    // 1) <key> 태그 읽어서 keyIdToName 맵핑
    for (auto* key = root->FirstChildElement(); key; key = key->NextSiblingElement()) {
        if (localName(key->Name()) != "key") continue;
        const char* id = key->Attribute("id");
        const char* attr = key->Attribute("attr.name");
        if (id && attr) keyIdToName[id] = attr;
    }

    // 2) <graph> 태그 찾기
    XMLElement* graph = nullptr;
    for (auto* c = root->FirstChildElement(); c; c = c->NextSiblingElement()) {
        if (localName(c->Name()) == "graph") graph = c;
    }
    if (!graph) graph = root; // fallback

    // 3) 노드 파싱
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
        adj[id]; // 인접 리스트 초기화
    }

    // 4) 엣지 파싱
    for (auto* e = graph->FirstChildElement(); e; e = e->NextSiblingElement()) {
        if (localName(e->Name()) != "edge") continue;

        const char* s = e->Attribute("source");
        const char* t = e->Attribute("target");
        if (!s || !t) continue;

        double length = NAN;
        string road;
        string oneway;

        // 간선의 data 파싱
        for (auto* d = e->FirstChildElement(); d; d = d->NextSiblingElement()) {
            if (localName(d->Name()) != "data") continue;

            const char* key = d->Attribute("key");
            const char* text = d->GetText();
            if (!key || !text) continue;

            string attr = keyIdToName[key];

            if (attr == "length" || key == string("d16"))
                length = atof(text);
            else if (attr == "name" || key == string("d13"))
                road = text;
            else if (attr == "oneway")
                oneway = text;
        }

        // 4-1) 길이 없으면 위경도 기반으로 추정
        if (!isfinite(length)) {
            if (nodes.count(s) && nodes.count(t))
                length = haversine(nodes[s].lat, nodes[s].lon, nodes[t].lat, nodes[t].lon);
            else length = 0.0;
        }

        // 4-2) 방향성 처리 (oneway가 true면 단방향)
        bool isOne = false;
        if (!oneway.empty()) {
            string v = oneway;
            transform(v.begin(), v.end(), v.begin(), ::tolower);
            if (v == "true" || v == "yes" || v == "1") isOne = true;
        }

        // 인접리스트 및 간선정보 저장
        adj[s].push_back({ t, length });
        edgeInfoMap[s][t] = { length, road };

        if (!isOne) {
            adj[t].push_back({ s, length });
            edgeInfoMap[t][s] = { length, road };
        }
    }
    return true;
}

/* =========================================================
   사용자 입력 (lat, lon)에 가장 가까운 노드 찾기
   - 허용 거리 : 20m
   ========================================================= */
string findNode(double lat, double lon) {
    string best = "";
    double minD = 1e18;

    for (auto& kv : nodes) {
        const Node& n = kv.second;
        double d = haversine(lat, lon, n.lat, n.lon);
        if (d < minD) { minD = d; best = n.id; }
    }
    return (minD <= 20.0) ? best : "";
}

/* =========================================================
   경로 길이 계산 (노드 리스트 → 전체 거리)
   ========================================================= */
double pathLength(const vector<string>& p) {
    double sum = 0;
    for (size_t i = 1; i < p.size(); i++)
        sum += edgeInfoMap[p[i - 1]][p[i]].length;
    return sum;
}

/* =========================================================
   Dijkstra 최단 경로 알고리즘
   ========================================================= */
vector<string> dijkstra(const string& start, const string& goal) {
    unordered_map<string, double> dist;
    unordered_map<string, string> prev;

    // 거리 초기화
    for (auto& kv : nodes) dist[kv.first] = 1e18;
    dist[start] = 0;

    // 최소 힙
    priority_queue<pair<double, string>,
        vector<pair<double, string>>,
        greater<>> pq;

    pq.push({ 0, start });

    while (!pq.empty()) {
        auto [cd, u] = pq.top();
        pq.pop();

        if (u == goal) break;

        for (auto& pr : adj[u]) {
            string v = pr.first;
            double w = pr.second;
            double lightDelay = 0;
            if (trafficLightDelay.count(v)) {
                lightDelay = trafficLightDelay[v];
            }
            if (dist[v] > cd + w) {
                dist[v] = cd + w;
                prev[v] = u;
                pq.push({ dist[v], v });
            }
        }
    }

    // 목적지 도달 불가
    if (dist[goal] >= 1e18) return {};

    // 경로 복원
    vector<string> path;
    for (string cur = goal; ; cur = prev[cur]) {
        path.push_back(cur);
        if (cur == start) break;
    }
    reverse(path.begin(), path.end());
    return path;
}

/* =========================================================
   Monte Carlo Random Path Sampling
   - 비교 실험용 (비최적 경로 탐색)
   - Dijkstra 알고리즘 성능 비교 목적
   ========================================================= */
vector<string> monteCarlo(const string& start, const string& goal, int M, int N) {
    // 랜덤 엔진 시드
    mt19937_64 rng(
        chrono::high_resolution_clock::now().time_since_epoch().count()
    );

    vector<string> best;
    int bestStep = 1e9;
    double bestLen = 1e18;

    // M번 반복 수행
    for (int i = 0; i < M; i++) {
        vector<string> path = { start };
        string cur = start;

        // N 스텝까지 무작위 탐색
        for (int st = 0; st < N; st++) {
            if (cur == goal) break;
            auto& nb = adj[cur];
            if (nb.empty()) break;

            uniform_int_distribution<int> pick(0, nb.size() - 1);
            string nxt = nb[pick(rng)].first;

            path.push_back(nxt);
            cur = nxt;
        }

        // 목적지 도달한 경우 최적 후보 업데이트
        if (cur == goal) {
            double len = pathLength(path);
            if ((int)path.size() < bestStep ||
                ((int)path.size() == bestStep && len < bestLen)) {
                bestStep = path.size();
                bestLen = len;
                best = path;
            }
        }
    }
    return best;
}

/* =========================================================
   출력용 : 노드들의 path → "a-b-c-d" 문자열
   ========================================================= */
string toDash(const vector<string>& p) {
    string s;
    for (size_t i = 0; i < p.size(); i++) {
        s += p[i];
        if (i + 1 < p.size()) s += "-";
    }
    return s;
}

/* =========================================================
   Main
   ========================================================= */
int main() {
    string file = "jongro.graphml";

    if (!loadGraphML(file)) {
        cout << "Graph load failed\n";
        return 0;
    }

    // 사용자 입력
    double slat, slon, dlat, dlon;
    cout << "Insert start position. ";
    cin >> slat >> slon;
    cout << "Insert destination position. ";
    cin >> dlat >> dlon;

    // 노드 매칭
    string s = findNode(slat, slon);
    string d = findNode(dlat, dlon);

    if (s.empty() || d.empty()) {
        cout << "Node not found\n";
        return 0;
    }
    int n;
    cout << "신호등 개수 입력: ";
    cin >> n;

    for (int i = 0; i < n; i++) {
        string node;
        double delay;
        cout << "노드 ID와 신호등 대기시간 입력: ";
        cin >> node >> delay;
        trafficLightDelay[node] = delay;
    }
    // Monte Carlo 탐색 (M=2000, N=1000)
    auto mc = monteCarlo(s, d, 2000, 1000);
    double mcLen = mc.empty() ? -1 : pathLength(mc);

    // Dijkstra 최단경로
    auto dj = dijkstra(s, d);
    double djLen = dj.empty() ? -1 : pathLength(dj);

    // 출력
    cout << fixed << setprecision(6);
    cout << "[Random Sampling] Path distance (m): " << mcLen << "\n";
    cout << "[Random Sampling] Vehicle route: " << toDash(mc) << "\n";
    cout << "[Dijkstra] Total distance + traffic delay (sec): " << djLen << "\n";
    cout << "[Dijkstra] Vehicle route: " << toDash(dj) << "\n";

    return 0;
}
