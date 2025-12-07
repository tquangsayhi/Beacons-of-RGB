// Datastructures.cc

#include "datastructures.hh"

#include <random>

std::minstd_rand rand_engine; // Reasonably quick pseudo-random generator

template <typename Type>
Type random_in_range(Type start, Type end)
{
    auto range = end-start;
    ++range;

    auto num = std::uniform_int_distribution<unsigned long int>(0, range-1)(rand_engine);

    return static_cast<Type>(start+num);
}

// Modify the code below to implement the functionality of the class.
// Also remove comments from the parameter names when you implement
// an operation (Commenting out parameter name prevents compiler from
// warning about unused parameters on operations you haven't yet implemented.)

Datastructures::Datastructures()
{
    // Write any initialization you need here

}

Datastructures::~Datastructures()
{
    // Write any cleanup you need here

}

bool Datastructures::add_beacon(BeaconID id, Name const& name, Coord xy, Color color)
{
    if (beacons_.find(id) != beacons_.end()) return false;
    beacons_[id] = {name,xy,color};
    return true;
}

int Datastructures::beacon_count()
{
    return beacons_.size();
}

void Datastructures::clear_beacons()
{
    beacons_.clear();
}

std::vector<BeaconID> Datastructures::all_beacons()
{
    std::vector<BeaconID> list;
    list.reserve(beacons_.size());
    for (const auto& beacon: beacons_)
    {
        list.push_back(beacon.first);
    }
    return list;
}

Name Datastructures::get_name(BeaconID beacon_id)
{
    auto it = beacons_.find(beacon_id);
    if (it == beacons_.end()) {
        return NO_NAME;
    }
    return it->second.name;
}

Coord Datastructures::get_coordinates(BeaconID beacon_id)
{
    auto it = beacons_.find(beacon_id);
    if (it == beacons_.end()) {
        return NO_COORD;
    }
    return it->second.coord;
}

Color Datastructures::get_color(BeaconID beacon_id)
{
    auto it = beacons_.find(beacon_id);
    if (it == beacons_.end()) {
        return NO_COLOR;
    }
    return it->second.color;
}

std::vector<BeaconID> Datastructures::beacons_alphabetically()
{
    std::vector<BeaconID> beacon_list;
    beacon_list.reserve(beacons_.size());
    for (const auto& pair : beacons_) {
            beacon_list.push_back(pair.first);
        }
    std::sort(beacon_list.begin(),beacon_list.end(),
              [this](const BeaconID& a, const BeaconID& b) {
                return beacons_.at(a).name < beacons_.at(b).name;
    });
    return beacon_list;
}

std::vector<BeaconID> Datastructures::beacons_brightness_increasing()
{
    std::vector<BeaconID> beacon_list;
    beacon_list.reserve(beacons_.size());
    for (const auto& pair : beacons_) {
            beacon_list.push_back(pair.first);
        }
    std::sort(beacon_list.begin(),beacon_list.end(),
              [this](const BeaconID& a, const BeaconID& b) {
        int color_a = beacons_.at(a).color.r * 3 + beacons_.at(a).color.g*6 + beacons_.at(a).color.b ;
        int color_b = beacons_.at(b).color.r * 3 + beacons_.at(b).color.g*6 + beacons_.at(b).color.b ;
        return color_a < color_b;
    });
    return beacon_list;
}

BeaconID Datastructures::min_brightness()
{
    if (beacons_.empty()) {
        return NO_BEACON;
    }
    auto it = beacons_.begin();
    BeaconID min_id = it->first;
    int min_val = it->second.color.r * 3 + it->second.color.g * 6 + it->second.color.b;

    for (const auto& pair : beacons_) {
        int current_val = pair.second.color.r * 3 + pair.second.color.g * 6 + pair.second.color.b;

        if (current_val < min_val) {
            min_val = current_val;
            min_id = pair.first;
        }
    }
    return min_id;
}

BeaconID Datastructures::max_brightness()
{
    if (beacons_.empty()) {
        return NO_BEACON;
    }
    auto it = beacons_.begin();
    BeaconID max_id = it->first;
    int max_val = it->second.color.r * 3 + it->second.color.g * 6 + it->second.color.b;

    for (const auto& pair : beacons_) {
        int current_val = pair.second.color.r * 3 + pair.second.color.g * 6 + pair.second.color.b;

        if (current_val > max_val) {
            max_val = current_val;
            max_id = pair.first;
        }
    }
    return max_id;
}

std::vector<BeaconID> Datastructures::find_beacons(Name const& name)
{
    std::vector<BeaconID> beacon_list;
    for (const auto& pair : beacons_) {
        if (pair.second.name == name) beacon_list.push_back(pair.first);
        }
    std::sort(beacon_list.begin(),beacon_list.end());
    return beacon_list;
}

bool Datastructures::change_beacon_name(BeaconID id, const Name& newname)
{
    auto it = beacons_.find(id);

    if (it == beacons_.end()) {
        return false;
    }

    it->second.name = newname;
    return true;
}

bool Datastructures::add_lightbeam(BeaconID sourceid, BeaconID targetid)
{
        auto source_iter = beacons_.find(sourceid);
        auto target_iter = beacons_.find(targetid);

        // Check if both exist
        if (source_iter == beacons_.end() || target_iter == beacons_.end()) {
            return false;
        }

        // Check if source is already sending light
        if (source_iter->second.target != NO_BEACON) {
            return false;
        }

        // Prevent Self-Loops
        if (sourceid == targetid) {
            return false;
        }
        // Prevent indirect loop
        BeaconID current_check = targetid;
        while (current_check != NO_BEACON)
        {
            if (current_check == sourceid) return false;
            current_check = beacons_.at(current_check).target;
        }
        source_iter->second.target = targetid;
        target_iter->second.sources.push_back(sourceid);
        return true;
}

std::vector<BeaconID> Datastructures::get_lightsources(BeaconID id)
{
    if(beacons_.find(id) == beacons_.end()){
        return {NO_BEACON};
    }
    std::vector<BeaconID> beacon_list = beacons_.at(id).sources;
    std::sort(beacon_list.begin(),beacon_list.end());
    return beacon_list;

}

std::vector<BeaconID> Datastructures::path_outbeam(BeaconID id)
{
    if(beacons_.find(id) == beacons_.end()){
        return {NO_BEACON};
    }
    std::vector<BeaconID> beacon_list;
    BeaconID current_check = id;
    while(current_check != NO_BEACON)
    {
        beacon_list.push_back(current_check);
        current_check = beacons_.at(current_check).target;
    }
    return beacon_list;
}

std::vector<BeaconID> Datastructures::path_inbeam_longest(BeaconID id)
{
    if (beacons_.find(id) == beacons_.end())
    {
        return {NO_BEACON};
    }
    std::vector<BeaconID> res;
    std::vector<BeaconID> cur;
    // recursion - call
    find_longest_path(id,res,cur);
    std::reverse(res.begin(), res.end());
    return res;
}

void Datastructures::find_longest_path(BeaconID ID, std::vector<BeaconID>& res, std::vector<BeaconID>& cur)
{
    cur.push_back(ID);
    if (beacons_.at(ID).sources.empty())
    {
        if (cur.size() > res.size())
        {
            res = cur;
        }
    }
    else {
        for (auto id: beacons_.at(ID).sources)
        {
            find_longest_path(id,res,cur);
            cur.pop_back();
        }
    }
}

Color Datastructures::total_color(BeaconID id)
{
    auto it = beacons_.find(id);
    if (it == beacons_.end()) {
        return NO_COLOR;
    }

    int total_r = it->second.color.r;
    int total_g = it->second.color.g;
    int total_b = it->second.color.b;

    int count = 1;

    for (BeaconID source_id : it->second.sources) {

        Color incoming = total_color(source_id);

        total_r += incoming.r;
        total_g += incoming.g;
        total_b += incoming.b;

        count++;
    }
    return { total_r / count, total_g / count, total_b / count };
}

bool Datastructures::add_fibre(Coord xpoint1, Coord xpoint2, Cost cost)
{
    if (xpoint1 == xpoint2)
    {
        return false;
    }
    auto it = fibers_.find(xpoint1);
    if (it != fibers_.end())
    {
        for (const auto& connection : it->second) {
            if (connection.first == xpoint2) {
                return false;
            }
        }
    }
    fibers_[xpoint1].push_back({xpoint2, cost});
    fibers_[xpoint2].push_back({xpoint1, cost});
    return true;

}

std::vector<Coord> Datastructures::all_xpoints()
{
    std::vector<Coord> res;
    res.reserve(fibers_.size());
    for (const auto& pair : fibers_) {
        res.push_back(pair.first);
    }
    return res;
}

std::vector<std::pair<Coord, Cost> > Datastructures::get_fibres_from(Coord xpoint)
{
    auto it = fibers_.find(xpoint);
    if (it == fibers_.end())
    {
        return {};
    }
    std::vector<std::pair<Coord, Cost> > res = it->second;
    std::sort(res.begin(), res.end());
    return res;
}

std::vector<std::pair<Coord, Coord> > Datastructures::all_fibres()
{
    std::vector<std::pair<Coord, Coord>> result;

    for (const auto& pair_outer : fibers_) {
        Coord start_node = pair_outer.first;
        const auto& connections = pair_outer.second;

        for (const auto& connection : connections) {
            Coord end_node = connection.first;
            if (start_node < end_node) {
                result.push_back({start_node, end_node});
            }
        }
    }
    std::sort(result.begin(), result.end());

    return result;
}

bool Datastructures::remove_fibre(Coord xpoint1, Coord xpoint2)
{
    auto it1 = fibers_.find(xpoint1);
    auto it2 = fibers_.find(xpoint2);

    if (it1 == fibers_.end() || it2 == fibers_.end()) {
        return false;
    }

    auto& vec1 = it1->second;

    auto new_end1 = std::remove_if(vec1.begin(), vec1.end(),
        [&](const auto& element) {
            return element.first == xpoint2;
        });

    if (new_end1 == vec1.end()) {
        return false;
    }

    vec1.erase(new_end1, vec1.end());

    auto& vec2 = it2->second;

    auto new_end2 = std::remove_if(vec2.begin(), vec2.end(),
        [&](const auto& element) {
            return element.first == xpoint1;
        });

    vec2.erase(new_end2, vec2.end());

    return true;

}

void Datastructures::clear_fibres()
{
    fibers_.clear();
}

std::vector<std::pair<Coord, Cost> > Datastructures::route_any(Coord fromxy, Coord toxy)
{
    // Check for Validation
    if (fibers_.find(fromxy) == fibers_.end() || fibers_.find(toxy) == fibers_.end()) {
        return {};
    }

    // Local Data Structures
    std::vector<std::pair<Coord, Cost>> path;
    std::set<Coord> visited;

    // DFS Lambda used as recursion
    // Local variable: Current Node, Cost of the edge we just traversed to get here
    std::function<bool(Coord, Cost)> dfs =
        [&](Coord current, Cost edge_cost) -> bool
    {
        // If path is empty (start node), cost is 0.
        // Otherwise, take the last node's total + this edge weight.
        Cost current_total = 0;
        if (!path.empty()) {
            current_total = path.back().second + edge_cost; // FIXED: Used path.back()
        }

        // Add to path and mark visited
        visited.insert(current);
        path.push_back({current, current_total});

        // Check Target
        if (current == toxy) {
            return true;
        }

        // Explore Neighbors
        for (const auto& edge : fibers_[current]) {
            Coord neighbor = edge.first;
            Cost next_edge_weight = edge.second;

            // Only visit if not already visited
            if (visited.find(neighbor) == visited.end()) {
                if (dfs(neighbor, next_edge_weight)) {
                    return true;
                }
            }
        }

        // If we get here, all neighbors failed.
        path.pop_back();
        return false;
    };

    // Start at 'fromxy' with an incoming edge cost of 0
    if (dfs(fromxy, 0)) {
        return path;
    }

    return {};

}

std::vector<std::pair<Coord, Cost>> Datastructures::route_least_xpoints(Coord fromxy, Coord toxy)
{
    // Check Validation if start/end points exist
    if (fibers_.find(fromxy) == fibers_.end() || fibers_.find(toxy) == fibers_.end()) {
        return {};
    }
    // BFS Setup
    std::queue<Coord> q;
    q.push(fromxy);

    // This map stores: key and its parent, where it came from
    // We also store the cost of that specific edge to make reconstruction easier.
    std::unordered_map<Coord, std::pair<Coord, Cost>,CoordHash> came_from;

    came_from[fromxy] = {fromxy, 0};

    bool found = false;
    while (!q.empty()) {
        Coord current = q.front();
        q.pop();

        if (current == toxy) {
            found = true;
            break;
        }
        // Check neighbors
        for (const auto& edge : fibers_[current]) {
            Coord neighbor = edge.first;
            Cost cost = edge.second + came_from[current].second;

            // If neighbor not visited yet
            if (came_from.find(neighbor) == came_from.end()) {
                came_from[neighbor] = {current, cost};
                q.push(neighbor);
                if (neighbor == toxy) {
                    found = true;
                    break;
                }
            }
        }
        if (found) {
            break;
        }
    }

    // Path Reconstruction

    if (!found) {
        return {};
    }

    std::vector<std::pair<Coord, Cost>> route;
    Coord curr = toxy;


    while (curr != fromxy) {
        auto data = came_from[curr];
        Coord parent = data.first;
        Cost cost = data.second;

        route.push_back({curr,cost});
        curr = parent;
    }
    route.push_back({fromxy,0});
    std::reverse(route.begin(), route.end());
    return route;

}

std::vector<std::pair<Coord, Cost>> Datastructures::route_fastest(Coord fromxpoint, Coord toxpoint)
{
    // Validation: Check if points exist in the graph
    if (fibers_.find(fromxpoint) == fibers_.end() || fibers_.find(toxpoint) == fibers_.end()) {
        return {};
    }

    if (fromxpoint == toxpoint) {
         return {{fromxpoint, 0}};
    }
    // Dijkstra Setup
    // Priority Queue: Stores {Cost, Coord}, ordered by smallest Cost first.
    using P = std::pair<Cost, Coord>;
    std::priority_queue<P, std::vector<P>, std::greater<P>> pq;

    std::unordered_map<Coord, Cost, CoordHash> dist;
    std::unordered_map<Coord, Coord, CoordHash> parent;

    // Initialize start node
    dist[fromxpoint] = 0;
    parent[fromxpoint] = fromxpoint;
    pq.push({0, fromxpoint});

    bool found = false;

    // The Search Loop
    while (!pq.empty()) {
        Cost current_cost = pq.top().first;
        Coord current = pq.top().second;
        pq.pop();

        if (current == toxpoint) {
            found = true;
            break;
        }

        // If the cost in the queue is worse than what we already found, skip it.
        if (current_cost > dist[current]) {
            continue;
        }

        // Explore neighbors
        for (const auto& edge : fibers_[current]) {
            Coord neighbor = edge.first;
            Cost edge_weight = edge.second;

            Cost new_dist = current_cost + edge_weight;

            // Relaxation Step:
            // Check if we found a new node OR a shorter path to an existing node.
            if (dist.find(neighbor) == dist.end() || new_dist < dist[neighbor]) {
                dist[neighbor] = new_dist;
                parent[neighbor] = current;
                pq.push({new_dist, neighbor});
            }
        }
    }

    // Path Reconstruction

    if (!found) {
        return {};
    }

    std::vector<std::pair<Coord, Cost>> route;
    Coord curr = toxpoint;

    // Backtrack from End to Start
    while (curr != fromxpoint) {
        route.push_back({curr, dist[curr]});
        curr = parent[curr];
    }

    // Add the start node (Cost is 0)
    route.push_back({fromxpoint, 0});

    // Reverse to get Start -> End order
    std::reverse(route.begin(), route.end());

    return route;
}

std::vector<Coord> Datastructures::route_fibre_cycle(Coord startxy)
{
    // Validation
    if (fibers_.find(startxy) == fibers_.end()) {
        return {};
    }

    //  Data Structures for DFS
    // 'path' stores the current route we are building
    std::vector<Coord> path;

    // 'visited' keeps track of nodes we have entered in the current search
    std::set<Coord> visited;

    // The Recursive DFS
    // Returns true if a cycle is found
    std::function<bool(Coord, Coord)> dfs =
        [&](Coord current, Coord parent) -> bool
    {
        visited.insert(current);
        path.push_back(current);

        // We must sort the neighbors before checking them.
        auto neighbors = fibers_[current];

        // Sort by Coordinate
        std::sort(neighbors.begin(), neighbors.end());

        for (const auto& edge : neighbors) {
            Coord neighbor = edge.first;

            if (neighbor == parent) {
                continue;
            }

            if (visited.count(neighbor)) {
                path.push_back(neighbor);
                return true;
            }

            // If not visited, recurse deeper
            if (dfs(neighbor, current)) {
                return true;
            }
        }

        path.pop_back();
        return false;
    };

    // Start the Search
    // We pass 'startxy' as current, and a  virtual coord as parent
    // to ensure we don't crash on the first step.
    if (dfs(startxy, { 999999, 999999 })) {
        return path;
    }

    return {}; // No cycle found
}
