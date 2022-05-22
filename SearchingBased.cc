#include <algorithm>
#include <cmath>
#include <iostream>
#include <queue>
#include <vector>
using namespace std;

class NodeMsg {
   public:
    NodeMsg* Pre;
    float MotionCost = numeric_limits<float>::max();
    int Obs = 0;
    int Draw = 0;
    int IsClosed = 0;
    int Visited = 0;
};

class Node {
   public:
    int x_;
    int y_;
    float FCost_;
    Node(int x, int y, float FCost) : x_(x), y_(y), FCost_(FCost) {}
};

class SearchingBasedPlanning {
   public:
    int EnvLong = 52;
    int EnvWide = 32;
    float HGain_ = 1;
    int CountVisited = 0;
    vector<Node*> MotionCostUpdated;
    pair<int, int> start_, end_;
    vector<vector<NodeMsg*>> NodeMap;
    struct NodeComp {
        bool operator()(Node* A, Node* B) { return A->FCost_ > B->FCost_; }
    };
    priority_queue<Node*, vector<Node*>, NodeComp>
        pri;  // TODO:(hong) use a min-heap
    vector<Node> Action{Node(1, 1, sqrt(2)),  Node(-1, -1, sqrt(2)),
                        Node(1, -1, sqrt(2)), Node(-1, 1, sqrt(2)),
                        Node(0, 1, 1),        Node(1, 0, 1),
                        Node(0, -1, 1),       Node(-1, 0, 1)};
    float heuristic(int x, int y) {
        return HGain_ * sqrt(pow(x - end_.first, 2) + pow(y - end_.second, 2));
    }
    float heuristic1(int x, int y) {
        auto tp = max(pow(x - end_.first, 2), pow(y - end_.second, 2));
        return HGain_ * sqrt(tp);
    }
    void Searching(int UseCloseList) {
        if (pri.empty()) {
            NodeMap[start_.first][start_.second]->MotionCost = 0;
            pri.push(new Node(start_.first, start_.second, 0));
        }
        while (true) {
            auto Cur = pri.top();
            if (NodeMap[Cur->x_][Cur->y_]->MotionCost >=
                NodeMap[end_.first][end_.second]->MotionCost) {
                return;
            }
            pri.pop();
            NodeMap[Cur->x_][Cur->y_]->IsClosed = 1;
            if (Cur->x_ == end_.first && Cur->y_ == end_.second) {
                return;
            }
            for (auto tp : Action) {
                auto Nx = Cur->x_ + tp.x_;
                auto Ny = Cur->y_ + tp.y_;
                if (1 == NodeMap[Nx][Ny]->Obs) {
                    continue;
                }
                if (NodeMap[Nx][Ny]->IsClosed && UseCloseList) {
                    continue;
                }
                if (NodeMap[Cur->x_][Cur->y_]->MotionCost + tp.FCost_ <
                    NodeMap[Nx][Ny]->MotionCost) {
                    CountVisited++;
                    NodeMap[Nx][Ny]->Draw = 8;
                    NodeMap[Nx][Ny]->Visited = 8;
                    NodeMap[Nx][Ny]->MotionCost =
                        NodeMap[Cur->x_][Cur->y_]->MotionCost + tp.FCost_;
                    NodeMap[Nx][Ny]->Pre = NodeMap[Cur->x_][Cur->y_];
                    if (NodeMap[Nx][Ny]->IsClosed) {
                        MotionCostUpdated.push_back(new Node(Nx, Ny, 0));
                        continue;
                    }

                    pri.push(new Node(Nx, Ny,
                                      NodeMap[Cur->x_][Cur->y_]->MotionCost +
                                          heuristic(Nx, Ny)));
                }
            }
        }
    }
    void ARAstar(pair<int, int> start, pair<int, int> end, int UseCloseList,
                 float HGain) {
        start_ = start;
        end_ = end;
        HGain_ = HGain;

        SetEnv();
        Searching(UseCloseList);
        vector<int> VisitedCamp;
        ClcPath();
        NodeMap[start_.first][start_.second]->Draw = 7;
        NodeMap[end_.first][end_.second]->Draw = 7;
        Animation();
        cout << HGain_ << ", " << CountVisited << endl;
        while (HGain_ > 1) {
            HGain_ = HGain_ - 0.4;
            while (!pri.empty()) {
                MotionCostUpdated.push_back(pri.top());
                pri.pop();
            }

            for (auto temp : MotionCostUpdated) {
                temp->FCost_ = NodeMap[temp->x_][temp->y_]->MotionCost +
                               heuristic(temp->x_, temp->y_);
                pri.push(temp);
            }
            if (pri.top()->FCost_ >
                NodeMap[end_.first][end_.second]->MotionCost) {
                break;
            }
            for (auto t1 : NodeMap) {
                for (auto t2 : t1) {
                    if (t2->Draw != 1) {
                        t2->Draw = 0;
                    }
                    t2->IsClosed = 0;
                    t2->Visited = 0;
                }
            }
            VisitedCamp.push_back(CountVisited);
            CountVisited = 0;
            Searching(UseCloseList);
            ClcPath();
            NodeMap[start_.first][start_.second]->Draw = 7;
            NodeMap[end_.first][end_.second]->Draw = 7;
            Animation();
            cout << HGain_ << ", " << CountVisited << endl;
        }
    }
    void Astar(pair<int, int> start, pair<int, int> end, int UseCloseList,
               int HGain) {
        start_ = start;
        end_ = end;
        HGain_ = HGain;
        SetEnv();
        Searching(UseCloseList);
        ClcPath();
        NodeMap[start_.first][start_.second]->Draw = 7;
        NodeMap[end_.first][end_.second]->Draw = 7;
        Animation();
    }
    void ClcPath() {
        auto temp = NodeMap[end_.first][end_.second];
        while (temp->Pre) {
            temp->Draw = 5;
            temp = temp->Pre;
        }
    }
    void SetEnv() {
        for (int i = 0; i < EnvWide; i++) {
            vector<NodeMsg*> temp(EnvLong);
            generate(temp.begin(), temp.end(), []() { return new NodeMsg; });
            NodeMap.push_back(temp);
        }
        for (int i = 1; i < 16; i++) {
            NodeMap[i][20]->Obs = 1;
            NodeMap[i][20]->Draw = 1;
        }
        for (int i = 1; i < 17; i++) {
            NodeMap[i + 14][30]->Obs = 1;
            NodeMap[i + 14][30]->Draw = 1;
        }
        for (int i = 0; i < EnvLong; i++) {
            NodeMap[0][i]->Obs = 1;
            NodeMap[EnvWide - 1][i]->Obs = 1;
            NodeMap[0][i]->Draw = 1;
            NodeMap[EnvWide - 1][i]->Draw = 1;
        }
        for (int i = 0; i < EnvWide; i++) {
            NodeMap[i][0]->Obs = 1;
            NodeMap[i][EnvLong - 1]->Obs = 1;
            NodeMap[i][0]->Draw = 1;
            NodeMap[i][EnvLong - 1]->Draw = 1;
        }
        for (int i = 10; i < 20; i++) {
            NodeMap[15][i]->Obs = 1;
            NodeMap[15][i]->Draw = 1;
        }
    }
    void Animation() {
        for (auto t1 : NodeMap) {
            for (auto t2 : t1) {
                if (t2->Draw == 0) {
                    cout << "  ";
                } else {
                    cout << t2->Draw << " ";
                }
            }
            cout << endl;
        }
    }
    void Animation(SearchingBasedPlanning* Other) {
        for (int i = 0; i < NodeMap.size(); i++) {
            for (int j = 0; j < NodeMap[0].size(); j++) {
                if (NodeMap[i][j]->Draw == 0 ||
                    (NodeMap[i][j]->Draw == 8 &&
                     Other->NodeMap[i][j]->Draw == 8)) {
                    cout << "  ";
                } else {
                    cout << NodeMap[i][j]->Draw << " ";
                }
            }
            cout << endl;
        }
    }
};

int main() {
    SearchingBasedPlanning test, test1, test2, test3, test4;
    test.Astar({5, 5}, {25, 45}, true, 10);
    cout << "----------------"
            "With Clost List"
            "---------------- "
         << "\n"
         << endl;
    test1.Astar({5, 5}, {25, 45}, false, 10);
    cout << "----------------"
            "with Clost but still update motion clost"
            "---------------- "
         << "\n"
         << endl;

    test1.Animation(&test);
    cout << "----------------"
            "difference"
            "---------------- "
         << "\n"
         << endl;
    test2.Astar({5, 5}, {25, 45}, false, 1);
    cout << "----------------"
            "With small heuristic"
            "---------------- "
         << "\n"
         << endl;
    test3.Astar({5, 5}, {25, 45}, true, 0);
    cout << "----------------"
            "Dijkstra"
            "---------------- "
         << "\n"
         << endl;

    test4.ARAstar({5, 5}, {25, 45}, false, 2.5);
    cout << "----------------"
            "ARAstar"
            "---------------- "
         << "\n"
         << endl;
    cout << "Done !" << endl;

    return 0;
}