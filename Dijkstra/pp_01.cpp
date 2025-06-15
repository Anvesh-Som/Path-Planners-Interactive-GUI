#include <boost/python/numpy.hpp>
#include <boost/scoped_array.hpp>
#include <iostream>
#include <boost/python.hpp>
#include <cstdint>
#include <string>
#include <vector>
#include <boost/utility.hpp>
#include <boost/shared_ptr.hpp>
#include <queue>
#include <map>
#include <utility>
#include <tuple>

namespace p = boost::python;
namespace np = boost::python::numpy;

using namespace std;

class dijkstra{
	public:

	dijkstra(int extent_x, int extent_y): world_extents_x(extent_x), world_extents_y(extent_y){

		world_obstacles = vector<vector<uint8_t>>(extent_x, vector<uint8_t>(extent_y));
	}
	
	void compute(){

		open_list = priority_queue<Node, vector<Node>, CompareCost>();
		visited = vector<vector<double>>();
		optimal_path = std::vector<vector<int>>();
		came_from = map<pair<int, int>, pair<int,int> >();

		visited = vector<vector<double>>(world_extents_x, vector<double>(world_extents_y));  //Initalizing the visited_nodes with zeros.

	    // List implementation -> find min O(k) but if the list is sorted as we make it(eg heap, it will take O(log k)		
		// Make a front list that is a tuple =(x,y,cost); Intially put the start and 0 cost in it.   
    	//Node(int x, int y, int prev_x, int prev_y, double cost)

		open_list.push(Node(start_x, start_y, -1, -1, 0.0)); 
		
		//		TODO: Add a dictionary named came_from = {}

		// while front is not empty
		while(!open_list.empty()){
 
		// 	get item with least cost and remove it from the front.    
		//	std::cout<<"start "<< start_x << " " << start_y << " ";        
		//	std::cout<<"goal "<< goal_x << " " << goal_y << " ";        
		//	std::cout<<"open_list "<< open_list.size() << " "; 

			Node element = open_list.top();
			open_list.pop();
		
			//check if it has already been visited.
			//if visited skip rest of loop by checking if value in visited array is greater than 0.
			if(visited[element.x][element.y] > 0 ) continue;
			 
			//set this value equal to the 1 or visited[pos] = cost 
			visited[element.x][element.y] = element.cost;                               

			//TODO: Set dictionary[current] = previous; came_from
			pair<int, int> curr(element.x, element.y);
			pair<int, int> prev(element.prev_x, element.prev_y);
			came_from.insert(make_pair(curr, prev));
			

			// if pos ==  goal break;
			if(element.x == goal_x && element.y == goal_y){

				int t_x = element.x;
				int t_y = element.y;
				optimal_path.push_back({t_x,t_y});
	
				while(t_x!=-1 && t_y!=-1){
	
					auto p2 = came_from.find(make_pair(t_x,t_y));
					//came_from[make_pair(t_x,t_y)];
					vector<int> vv = {p2->second.first,p2->second.second};
					if(vv[0] > 0 && vv[1] >0)
						optimal_path.push_back(vv);
					t_x = p2->second.first;
					t_y = p2->second.second;
				}

				std::reverse(optimal_path.begin(), optimal_path.end());
				/*for(int i=0; i<optimal_path.size();i++){
					std::cout<<optimal_path[i][0]<<" "<<optimal_path[i][1]<<"      ";
				}
				*/
				break;
			} 

			// for all items in the movements 
			for(auto i=0; i<movements.size(); i++){

				move m = movements[i];

				// - Compute new_x and new_y from old position 'pos' and dx, dy.
				int new_x = element.x + m.dx;
				int new_y = element.y + m.dy;
				double new_cost = element.cost + m.cost;
				// - Check that new_x is >= 0 and < extents[0], similarly for new_y.
				// - If not, skip the remaining part of this loop.
				if((new_x < 0 || new_y < 0) || (new_x > world_extents_x || new_y > world_extents_y)) continue;
        	
				// if visited is zero and obstacle is not 255 then append the new_x, new_y and new_cost to the list   
				if(world_obstacles[new_x][new_y] ==0 && visited[new_x][new_y] == 0){
					
					Node n = Node(new_x, new_y, element.x, element.y , new_cost);
					open_list.push(n);

				}
			
			}
    	
  		}
		
	}

	void return_visited(p::object obj){  //Return the visited array .

		PyObject* pobj = obj.ptr();
    	Py_buffer pybuf;
    	PyObject_GetBuffer(pobj, &pybuf, PyBUF_SIMPLE);
    	void *buf = pybuf.buf;
    	double *p = (double*)buf;
    	Py_XDECREF(pobj);

    	u_int n_rows = visited.size();
    	u_int n_cols = visited[0].size();
    	for (u_int i = 0; i < n_rows; i++)
    	{
        	for (u_int j = 0; j < n_cols; j++)
        	{ 
            	p[i*n_cols+j] = visited[i][j];
        	}
    	}
	}

	p::object get_path_dim1()
	{
  		using boost::python::object;
  		return object(optimal_path.size());
	}
	p::object get_path_dim2()
	{
  		using boost::python::object;
  		return object(optimal_path[0].size());
	}


	void return_path(p::object obj){  //TODO: Return the path.

		PyObject* pobj = obj.ptr();
    	Py_buffer pybuf;
    	PyObject_GetBuffer(pobj, &pybuf, PyBUF_SIMPLE);
    	void *buf = pybuf.buf;
    	int *p = (int*)buf;
    	Py_XDECREF(pobj);

    	u_int n_rows = optimal_path.size();
    	u_int n_cols = optimal_path[0].size();
    	for (u_int i = 0; i < n_rows; i++)
    	{
        	for (u_int j = 0; j < n_cols; j++)
        	{
            	p[i*n_cols+j] = optimal_path[i][j];
        	}
    	}

	}


	void fill_obstacles(np::ndarray const & input) {
    	
		int rows = input.shape(0);
    	int cols = input.shape(1);
    	uint8_t* input_ptr = reinterpret_cast<uint8_t*>(input.get_data());
		uint8_t row_stride = input.strides(0) / sizeof(uint8_t); //No of elements in a row.
	    uint8_t col_stride = input.strides(1) / sizeof(uint8_t); //No of elements in a col.
		
		uint8_t *row_iter = input_ptr;
		for (int i = 0; i < rows; ++i, row_iter += row_stride) { //row_stride added to move to next row after accessing all the cols of each row.
        	uint8_t * col_iter = row_iter;
        	for (int j = 0; j < cols; ++j, col_iter += col_stride) { //col_stride added as elements that are in input are stored in col_major order.
            	world_obstacles[i][j] = *col_iter;
        	}
    	}
	
	}

	

	void fill_start_goal(int start_x, int start_y, int goal_x, int goal_y){
		
		this->start_x = start_x;
		this->start_y = start_y;
		this->goal_x = goal_x;	
		this->goal_y = goal_y;	

	}

	int world_extents_x, world_extents_y, start_x,start_y, goal_x, goal_y; //Get it passed as param;
	vector<vector<uint8_t>> world_obstacles; //Get it passed as param //check this needs to be updated to add obstacles.
	vector<vector<double>> visited;
	vector<vector<int>> optimal_path;
	map<pair<int, int>, pair<int,int> > came_from;

	//---------------------------------------------------------------------------------------------------------------------------------------------	
	struct Node {
 		int x;
		int y;
		int prev_x;
		int prev_y;
		double cost;
		
    	// this will used to initialize the variables
    	// of the structure
    	Node(int x_, int y_, int prev_x_, int prev_y_, double cost_)
        	: x(x_), y(y_), prev_x(prev_x_), prev_y(prev_y_), cost(cost_) {}
	};
 
	// this is an structure which implements the
	// operator overloading
	struct CompareCost {
    	bool operator()(Node const& n1, Node const& n2)
    	{
        	return n1.cost > n2.cost; //Implementation of in_heap (min at top)
   	 	}
	};

	priority_queue<Node, vector<Node>, CompareCost> open_list;

	//-------------------------------------------------------------------------------------------------------------------------------------------------
		
	struct move{
		int dx;
		int dy;
		double cost;

		move(int x, int y, double c){
			dx = x;
			dy = y;
			cost = c;
		}
	};
	
	double s2 = 1.41421356237;
	vector<move> movements = { move(1,0, 1.0), move(0,1, 1.0), move(-1,0, 1.0), move(0,-1, 1.0),
              move(1,1, s2), move(-1,1, s2), move(-1,-1, s2), move(1,-1, s2)};

};




BOOST_PYTHON_MODULE(pp_01) {
    // An established convention for using boost.python.
    using namespace boost::python;
    Py_Initialize();
    np::initialize();

	class_<dijkstra>("dijkstra", 
		init<int const &, int const &>())
        .def("fill_obstacles", &dijkstra::fill_obstacles)
        .def("fill_start_goal", &dijkstra::fill_start_goal)
		.def("return_visited", &dijkstra::return_visited)
		.def("return_path", &dijkstra::return_path)
		.def("compute", &dijkstra::compute)
		.def("get_path_dim1", &dijkstra::get_path_dim1)
		.def("get_path_dim2", &dijkstra::get_path_dim2)
    ;

}
