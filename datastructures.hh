// Datastructures.hh

#ifndef DATASTRUCTURES_HH
#define DATASTRUCTURES_HH

#include <string>
#include <vector>
#include <utility>
#include <limits>
#include <source_location>
#include <unordered_map>
#include <algorithm>
#include <map>
#include <queue>
#include <functional>
#include <set>

// Type for beacon IDs
using BeaconID = std::string;
using Name = std::string;

// Return value for cases where required beacon was not found
BeaconID const NO_BEACON= "--NO_BEACON--";

// Return value for cases where integer values were not found
int const NO_VALUE = std::numeric_limits<int>::min();

// Return value for cases where name values were not found
Name const NO_NAME = "-- NO_NAME --";

// Type for a coordinate (x, y)
struct Coord
{
    int x = NO_VALUE;
    int y = NO_VALUE;
};

// Example: Defining == and hash function for Coord so that it can be used
// as key for std::unordered_map/set, if needed
inline bool operator==(Coord c1, Coord c2) { return c1.x == c2.x && c1.y == c2.y; }
inline bool operator!=(Coord c1, Coord c2) { return !(c1==c2); } // Not strictly necessary

struct CoordHash
{
    std::size_t operator()(Coord xy) const
    {
        auto hasher = std::hash<int>();
        auto xhash = hasher(xy.x);
        auto yhash = hasher(xy.y);
        // Combine hash values (magic!)
        return xhash ^ (yhash + 0x9e3779b9 + (xhash << 6) + (xhash >> 2));
    }
};

// Example: Defining < for Coord so that it can be used
// as key for std::map/set
inline bool operator<(Coord c1, Coord c2)
{
    if (c1.y < c2.y) { return true; }
    else if (c2.y < c1.y) { return false; }
    else { return c1.x < c2.x; }
}

// Return value for cases where coordinates were not found
Coord const NO_COORD = {NO_VALUE, NO_VALUE};

// Type for color (RGB)
struct Color
{
    int r = NO_VALUE;
    int g = NO_VALUE;
    int b = NO_VALUE;
};

// Equality and non-equality comparisons for Colors
inline bool operator==(Color c1, Color c2) { return c1.r == c2.r && c1.g == c2.g && c1.b == c2.b; }
inline bool operator!=(Color c1, Color c2) { return !(c1==c2); }

// Return value for cases where color was not found
Color const NO_COLOR = {NO_VALUE, NO_VALUE, NO_VALUE};

// Type for light transmission cost (used only in the second assignment)
using Cost = int;

// Return value for cases where cost is unknown
Cost const NO_COST = NO_VALUE;

// This exception class is there just so that the user interface can notify
// about operations which are not (yet) implemented
class NotImplemented : public std::exception
{
public:
    explicit NotImplemented(std::string const msg = "",
                            const std::source_location location = std::source_location::current())
        : msg_{}
    {
        std::string funcname = location.function_name();
        if (auto namestart = funcname.find_last_of(':'); namestart != std::string::npos)
        { funcname.erase(0, namestart+1); }
        if (auto nameend = funcname.find_first_of('('); nameend != std::string::npos)
        { funcname.erase(nameend, std::string::npos); }
        msg_ = (!msg.empty() ? msg+" in " : "")+funcname+"()";
    }
    virtual const char* what() const noexcept override
    {
        return msg_.c_str();
    }
private:
    std::string msg_;
};

// This is the class you are supposed to implement
struct Beacon {
  Name name;
  Coord coord;
  Color color;
  BeaconID target = NO_BEACON;
  std::vector<BeaconID> sources;
};

class Datastructures
{
public:
    Datastructures();
    ~Datastructures();

    // A operations

    // Estimate of performance: O(1) average
    // Short rationale for estimate:add_beacon first checks whether the ID already exists using unordered_map::contains, which is O(1) average.
    // Inserting a new element with operator[] is also O(1) average. Therefore, the whole operation runs in constant average time O(1)
    bool add_beacon(BeaconID id, Name const& name, Coord xy, Color color);

    // Estimate of performance: O(1)
    // Short rationale for estimate:the map tracks its size without needing to traverse elements.
    int beacon_count();

    // Estimate of performance: O(n)
    // Short rationale for estimate: Each stored element must be destroyed, but no rehashing or lookup is required.
    //After clearing, the container size becomes zero.
    void clear_beacons();

    // Estimate of performance: O(n)
    // Short rationale for estimate: The function just loop throught the unordered map once
    std::vector<BeaconID> all_beacons();

    // Estimate of performance:O(1)
    // Short rationale for estimate: It runs in average O(1) time because the hash map jumps directly to the data location
    Name get_name(BeaconID id);

    // Estimate of performance:O(1)
    // Short rationale for estimate: It runs in average O(1) time because the hash map jumps directly to the data location
    Coord get_coordinates(BeaconID id);

    // Estimate of performance:O(1)
    // Short rationale for estimate: It runs in average O(1) time because the hash map jumps directly to the data location
    Color get_color(BeaconID id);

    // We recommend you implement the operations below only after implementing the ones above

    // Estimate of performance: O(n log n)
    // Short rationale for estimate: The function first copies all beacon IDs into a vector in O(n) time.
    //It then sorts the vector using std::sort, which performs O(n log n) comparisons, and each comparison retrieves names in O(1) average time.
    //Therefore, the total running time is dominated by the sorting step, giving O(n log n) overall.
    std::vector<BeaconID> beacons_alphabetically();

    // Estimate of performance: O(n log n)
    // Short rationale for estimate: The function first copies all beacon IDs into a vector in O(n) time.
    //It then sorts the vector using std::sort, which performs O(n log n) comparisons, and each comparison retrieves colorss in O(1) average time.
    //Therefore, the total running time is dominated by the sorting step, giving O(n log n) overall.
    std::vector<BeaconID> beacons_brightness_increasing();

    // Estimate of performance: O(n)
    // Short rationale for estimate:It only goes through one loop
    BeaconID min_brightness();

    // Estimate of performance: O(n)
    // Short rationale for estimate:It only goes through one loop
    BeaconID max_brightness();

    // Estimate of performance: O(N) to search +O(M log M) to sort (where M is the number of found beacons)
    // Short rationale for estimate: O(N) to go through the data and O(M logM) to perform the sorting
    std::vector<BeaconID> find_beacons(Name const& name);

    // Estimate of performance:O(1)
    // Short rationale for estimate: the algorithm find cost only O(1)
    bool change_beacon_name(BeaconID id, Name const& newname);

    // We recommend you implement the operations below only after implementing the ones above

    // Estimate of performance:O(N)
    // Short rationale for estimate: Due to the cycle detection loop.
    bool add_lightbeam(BeaconID sourceid, BeaconID targetid);

    // Estimate of performance:O(N)
    // Short rationale for estimate: Only one loop through the data
    std::vector<BeaconID> get_lightsources(BeaconID id);

    // Estimate of performance: O(N)
    // Short rationale for estimate: Due to the loop of light rays
    std::vector<BeaconID> path_outbeam(BeaconID id);

    // B operations

    // Estimate of performance: O(N)
    // Short rationale for estimate: using the concept of tree traversal
    std::vector<BeaconID> path_inbeam_longest(BeaconID id);
    //helper function for pth_inbeam_longest
    void find_longest_path(BeaconID ID, std::vector<BeaconID>& res, std::vector<BeaconID>& cur);

    // Estimate of performance: O(N)
    // Short rationale for estimate: It follows tree structure and constant work per node.
    Color total_color(BeaconID id);

    // Estimate of performance: O(1)
    // Short rationale for estimate: The algorothm find costs only constant time as every Coord only shows up once in the data
    bool add_fibre(Coord xpoint1, Coord xpoint2, Cost cost);

    // Estimate of performance: O(N)
    // Short rationale for estimate: It loops through the data once
    std::vector<Coord> all_xpoints();

    // Estimate of performance: O(D logD) (D is the number of connections attched to xpoint)
    // Short rationale for estimate: This is strictly determined by the std::sort operation, which is computationally heavier than the lookup or copy steps.
    std::vector<std::pair<Coord, Cost>> get_fibres_from(Coord xpoint);

    // Estimate of performance: O(N logN)
    // Short rationale for estimate: The function goes through the data once, which take O(N)
    // However, the other sort cost O(N logN) in the end.
    std::vector<std::pair<Coord, Coord>> all_fibres();

    // Estimate of performance: O(D) where D is the number of connections to the point
    // Short rationale for estimate: std::remove_if must iterate through the entire list of connections (the vector) to find the specific target and then shift all subsequent elements to close the gap.
    //Since this linear scan and shift must be performed for both endpoints, the operation's speed depends directly on how many connections those points have.
    bool remove_fibre(Coord xpoint1, Coord xpoint2);

    // Estimate of performance: O(N)
    // Short rationale for estimate:
    void clear_fibres();

    // We recommend you implement the operations below only after implementing the ones above

    // Estimate of performance: O((N + E) log N), where N is the number of coordinates (nodes) and E is the number of fibers (edges).
    // Short rationale for estimate: Standard DFS is usually linear (O(N+E)), but because this implementation uses a std::set to track visited nodes, every check and insertion involves a tree search.
    // This adds a logarithmic overhead (O(log N)) to every step of the traversal.
    std::vector<std::pair<Coord, Cost>> route_any(Coord fromxpoint, Coord toxpoint);

    // C operations

    // Estimate of performance: O(N + D) where N is the number of nodes (coordinates) and D is the number of fibers (edges)
    // Short rationale for estimate: The function performs a standard Breadth-First Search, which ensures that each reachable node is enqueued and processed at most once
    //Since came_from is implemented as a std::unordered_map, the operations to check and record visited nodes take constant time O(1)
    std::vector<std::pair<Coord, Cost>> route_least_xpoints(Coord fromxpoint, Coord toxpoint);

     // Estimate of performance: O(N logN)
    // Short rationale for estimate: The function uses Dijkstra method.
    // The algorithm iterates through every fiber connection N) and potentially adds a new path to the priority queue.
    //Since every insertion into the priority queue takes logarithmic time O(log N), the total time is simply the number of edges multiplied by the cost of the queue operation.
    std::vector<std::pair<Coord, Cost>> route_fastest(Coord fromxpoint, Coord toxpoint);

    // Estimate of performance: O(N logN)
    // Short rationale for estimate: This is driven by two logarithmic operations performed during the traversal:
    //sorting the list of neighbors at every node, and using a std::set (a tree structure) to check if a node has been visited.
    std::vector<Coord> route_fibre_cycle(Coord startxpoint);

private:
    // Explain below your rationale for choosing the data structures you use in this class.

    // The main datastructure will be an unordered map by Beacon ID as key.
    // This is because the ID is unique and the structure provide fast look up for getting data
    std::unordered_map<BeaconID,Beacon> beacons_;

    std::map<Coord, std::vector<std::pair<Coord,Cost>>> fibers_;

    // Add stuff needed for your class implementation below

};

#endif // DATASTRUCTURES_HH
