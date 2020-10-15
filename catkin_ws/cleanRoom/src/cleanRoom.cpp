#include<iostream>
#include<vector>
#include <utility> 
#include <map> 
#include <queue> 
#include <stack>
#include <functional>
#include <chrono>
#include <iomanip>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <nav_msgs/GetMap.h>
#include <tf/transform_datatypes.h>
#include "rosbag/bag.h"
#include <rosbag/view.h>
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include <math.h>
#include <stdio.h> 
#include <time.h> 

#define INF 1e9
using namespace std;



# define obscall_rang 4
# define cover_rang 5

Mat convert_ocuGrid_img(nav_msgs::OccupancyGridConstPtr grid)
{
  GridMap2D gridmap (grid,false);
  Mat img=gridmap.binaryMap();
  return img;
}

vector<Point> calc_centroids(vector<vector<Point>> contours)
{
    vector<Point> result;
    Moments M;
    double cx,cy;
    for(int i=0;i<contours.size();i++)
    {
        M=moments(contours[i]);
        cx=M.m10/M.m00;
        cy=M.m01/M.m00;
        result.push_back(Point(int(cx),int(cy)));
    }
    return result;
}

void print_centroids(vector<Point> cen,int i)
{
    cout<<"centroids"<<i;
    for(int i=0;i<cen.size();i++)
    {
        cout<<"["<<cen[i].x <<", "<<cen[i].y<<"]"<<", ";
    }
    cout<<endl;
}
bool is_map_changed(nav_msgs::OccupancyGrid  new_grid)
{
    Mat old_image,new_image,old_dilt,new_dilt,old_bin,new_bin,kernel;
    vector<vector<Point>> old_contours,new_contours;
    vector<Point> old_centroids,new_centroids;
    nav_msgs::OccupancyGridConstPtr new_ptr=boost::shared_ptr< ::nav_msgs::OccupancyGrid const>(&new_grid);


    new_image = convert_ocuGrid_img(new_ptr);
    old_image = convert_ocuGrid_img(bag_pointer);


    
    threshold(old_image,old_bin,50,255,THRESH_BINARY_INV);
    threshold(new_image,new_bin,50,255,THRESH_BINARY_INV);

    kernel=getStructuringElement(MORPH_RECT,Size(3,3));
    dilate(old_bin,old_dilt,kernel);
    dilate(new_bin,new_dilt,kernel);

    findContours(old_dilt,old_contours,RETR_TREE,CHAIN_APPROX_NONE);
    findContours(new_dilt,new_contours,RETR_TREE,CHAIN_APPROX_NONE);

    old_centroids=calc_centroids(old_contours);
    new_centroids=calc_centroids(new_contours);

    print_centroids(old_centroids,1);
    print_centroids(new_centroids,2);
    if(old_centroids.size()!=new_centroids.size())
        return true;
    else
    {
        for(int i=0;i<old_centroids.size();i++)
        {
            if(abs(old_centroids[i].x-new_centroids[i].x)>3||abs(old_centroids[i].y-new_centroids[i].y)>3)
                return true;
        }
        return false;
    }
    

}

bool are_they_on_same_line(pair<int, int>n1, pair<int, int>n2, pair<int, int>n3) {
	if ((n1.first == n2.first && n2.first == n3.first && n1.first == n3.first) || (n1.second == n2.second && n2.second == n3.second && n1.second == n3.second)) {
		return true;
	}
	return false;
}
bool are_they_on_same_line2node(pair<int, int>n1, pair<int, int>n2) {
	if ((n1.first == n2.first) || (n1.second == n2.second)) {
		return true;
	}
	return false;
}

nav_msgs::OccupancyGrid result;
string map_path;
vector<vector<pair<int, int>>> matrix(6);
map< int, pair<int, int>> m;

void delay(int number_of_seconds)
{
	// Converting time into milli_seconds 
	int milli_seconds = 1000 * number_of_seconds;

	// Storing start time 
	clock_t start_time = clock();

	// looping till required time is not achieved 
	while (clock() < start_time + milli_seconds)
		;
}
vector<vector<pair<int, int>>> adj_list;
vector <int> check(6, 0);

int find(vector<int> p) {
	for (int i = 0; i < p.size(); i++) {

		if (p[i] == 1 || p[i] == 2) {
			return i;
		}

	}
	return -1;
}



pair<vector<int>, vector<int> > shortest_distance(int src
) {

	// Create queue
	priority_queue<pair<int, pair<int, int> >,
		vector<pair<int, pair<int, int>> >,
		greater<pair<int, pair<int, int> > > >nodes_q;

	// Create d and p arrays
	int n = adj_list.size();
	vector<int> d(n, INF);
	vector<int> p(n, -1);

	nodes_q.push({ 0, {src, src} });

	while (!nodes_q.empty()) {
		pair<int, pair<int, int> > cur_p = nodes_q.top();
		nodes_q.pop();
		int cur_node = cur_p.second.first;
		int cur_prev_node = cur_p.second.second;
		int cur_dis = cur_p.first;

		if (d[cur_node] != INF)
			continue;

		d[cur_node] = cur_dis;
		p[cur_node] = cur_prev_node;

		// Add the nodes connected to current one
		for (int i = 0;
			i < adj_list[cur_node].size();
			i++)
		{
			int next_node = adj_list[cur_node][i].first;
			int weight = adj_list[cur_node][i].second;
			if (d[next_node] != INF)
				continue;
			nodes_q.push({ cur_dis + weight,
						  {next_node, cur_node} });
		}
	}

	return { d, p };
}

vector<int> Get_Path(int src, int des, vector<int> p) {


	stack<int> path_nodes;
	int node2 = des;

	path_nodes.push(node2);
	vector<int>path;

	while (p[node2] != node2) {
		node2 = p[node2];
		path_nodes.push(node2);
	}


	while (!path_nodes.empty()) {

		path.push_back(path_nodes.top());
		path_nodes.pop();
	}
	return path;
}



double distance2N(pair<double, double> node1, pair<double, double> node2) {
	return sqrt(abs(node1.first - node2.first) * abs(node1.first - node2.first) +
		abs(node1.second - node2.second) * abs(node1.second - node2.second));

}




bool check_point(int x, int y, int range) {
	for (int i = -range; i <= range; i++) {
		for (int j = -range; j <= range; j++) {
			if (x + i < matrix.size() && y + j < matrix.size() && x - i >= 0 && y - j >= 0)
			{

				if (matrix[x + i][y + j].second == -1)
					return false;
			}

		}
	}
	return true;


}


pair<int, int> Nearest(vector<int> unVisitList, int node_num) {

	pair<int, int>  coor = m[node_num];
	int x = coor.first;
	int y = coor.second;
	vector<int> direction_queue;
	int min = INF;


	double min_distance2 = INF;
	double min_distance = INF;
	pair<vector<int>, vector<int>> res;
	vector<int>path2;
	res = shortest_distance(node_num);
	ROS_INFO("finish shortest_distance");
	path2 = res.first;
	vector<int> path;
	double calcu_dis;
	for (int i = 0; i < unVisitList.size(); i++) {

		if (unVisitList[i] == -1 || unVisitList[i] == 1 || unVisitList[i] == 2) {
			continue;
		}


		if (!check_point(m[i].first, m[i].second, obscall_rang)) {
			check[i] = 2;
			continue;

		}



		if (path2[i] < min_distance) {

			min_distance = path2[i];

			min = i;

		}

	}
	min_distance = distance2N(m[min], m[node_num]);

	for (int i = 0; i < path2.size(); i++) {
		if (path2[i] == INF && check[i] == 0) {
			min_distance2 = distance2N(m[i], m[node_num]);
			if (min_distance2 < min_distance) {
				min = i;
				min_distance = distance2N(m[min], m[node_num]);
			}

		}


	}


	if (min_distance == INF || min == INF) {
		double d;
		for (int i = 0; i < unVisitList.size(); i++) {
			if (unVisitList[i] != 0)
				continue;
			d = distance2N(m[i], m[node_num]);
			if (d < min_distance) {
				min_distance = d;
				min = i;
			}
		}

	}
	/*if (path2[min] == INF) {
		check[min] = 1;
		return -1;
	}*/
	ROS_INFO("finish loop");
	int size;
	if (min != INF || path2[min] != INF)
		size = Get_Path(node_num, min, res.second).size();
	else {
		size = 0;
	}



	ROS_INFO("dikstra finish min is %d", min);
	return { min,size };
}

void cover(int node_num, int range, int value) {
	pair<int, int>  coor = m[node_num];
	int x = coor.first;
	int y = coor.second;
	ROS_INFO("x=%d y=%d", x, y);

	for (int i = -range; i <= range; i++) {
		for (int j = -range; j <= range; j++) {
			if (x + i < matrix.size() && y + j < matrix.size() && x - i >= 0 && y - j >= 0)
			{
				check[matrix[x + i][y + j].first] = value;
			}

		}
	}



}
int Nearest2(vector<int> unVisitList, int node_num) {

	pair<int, int>  coor = m[node_num];
	int x = coor.first;
	int y = coor.second;
	vector<int> direction_queue;
	int min = INF;


	double min_distance2 = INF;
	double min_distance = INF;
	pair<vector<int>, vector<int>> res;
	vector<int>path2;
	res = shortest_distance(node_num);
	path2 = res.first;
	vector<int> path;
	double calcu_dis;
	for (int i = 0; i < unVisitList.size(); i++) {
		if (unVisitList[i] == 0) {
			continue;
		}





		if (path2[i] < min_distance) {

			min_distance = path2[i];

			min = i;

		}

	}



	if (min_distance == INF || min == INF) {
		double d;
		for (int i = 0; i < unVisitList.size(); i++) {
			if (unVisitList[i] == 0)
				continue;
			d = distance2N(m[i], m[node_num]);
			if (d < min_distance) {
				min_distance = d;
				min = i;
			}
		}

	}


	return min;
}


void update_adj_list(int next_node) {
	pair<int, int>  coor = m[next_node];
	int x = coor.first;
	int y = coor.second;

	if (x + 1 != matrix.size()) {
		if (matrix[x + 1][y].second == 1 && matrix[x][y].second != -1) {

			adj_list[matrix[x][y].first].push_back(matrix[x + 1][y]);
			adj_list[matrix[x + 1][y].first].push_back(matrix[x][y]);
		}
	}


	if (y + 1 != matrix[x].size()) {

		if (matrix[x][y + 1].second == 1 && matrix[x][y].second != -1) {

			adj_list[matrix[x][y].first].push_back(matrix[x][y + 1]);
			adj_list[matrix[x][y + 1].first].push_back(matrix[x][y]);
		}

	}




}

void update(int next_node, nav_msgs::OccupancyGrid  current_map, int range) {
	int   current_rows = current_map.info.height;
	pair<int, int>  coor = m[next_node];
	int x = coor.first;
	int y = coor.second;


	for (int i = -range; i <= range; i++) {
		for (int j = -range; j <= range; j++) {
			if (x + i < matrix.size() && y + j < matrix.size() && x - i >= 0 && y - j >= 0)
			{

				if (current_map.data[(x + i) + (y + j) * current_rows] == 0 && matrix[x + i][y + j].second == -1) { // unoccupied cell
					matrix[x + i][y + j].second = 1;
					if (check[matrix[x + i][y + j].first] == -1 || check[matrix[x + i][y + j].first] == 2) {
						check[matrix[x + i][y + j].first] = 0;
						cover(matrix[x + i][y + j].first, 5, 0);
					}
					if (result.data[(x + i) + (y + j) * current_rows] != 0 || result.data[(x + i) + (y + j) * current_rows] == 100) {

						check[matrix[x + i][y + j].first] = 0;

						update_adj_list(matrix[x + i][y + j].first);

					}
				}
				else if (current_map.data[(x + i) + (y + j) * current_rows] != 0 && matrix[x + i][y + j].second == 1) {
					matrix[x + i][y + j].second = -1;
					check[matrix[x + i][y + j].first] = -1;
					//    cover(matrix[x+i][y+j].first,8,2);
				   // erase_from_adjs_list(x+i,y+j);

				}
			}

		}
	}

}


vector<vector<int> > grid;
void readMap(char* path);
void save_map(ros::NodeHandle nh, char* path);

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int main(int argc, char** argv) {
	float current_x, current_y; // current pose
	nav_msgs::OccupancyGrid  current_map;
	ros::init(argc, argv, "navigation_goals");

	ros::NodeHandle nh;

	ROS_INFO("%s", argv[1]);
	readMap(argv[1]);


	ros::Rate rate(1);
	map_path = argv[1];


	tf::TransformListener listener;
	tf::StampedTransform transform;
	int grid_x;
	int grid_y;
	float map_x, map_y;

	clock_t  start, end;
	int num = result.info.width;
	matrix.resize(num);

	int t;



	int num_node = 0;

	int numPoints = num * num;


	check.resize(numPoints);//o(1)

	start = clock();
	//o(n)
	for (int i = 0; i < num; i++) {
		for (int j = 0; j < num; j++) {

			t = grid[i][j];
			m.insert({ num_node,{i,j} });
			if (t == 0) {

				matrix[i].push_back({ num_node,1 });

			}
			else {

				check[num_node] = -1;
				matrix[i].push_back({ num_node,-1 });

			}


			num_node++;

		}

	}





	//ROS_INFO("Attempting to read pose...");
	listener.waitForTransform("/map", "/base_link", ros::Time(), ros::Duration(100));
	listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
	map_x = transform.getOrigin().x();
	map_y = transform.getOrigin().y();
	grid_x = (unsigned int)((map_x - result.info.origin.position.x) / result.info.resolution);
	grid_y = (unsigned int)((map_y - result.info.origin.position.y) / result.info.resolution);
	ROS_INFO("Got a index! x = %d, y = %d", grid_x, grid_y);

	double theta_begin = atan2(map_y, map_x);
	// convert angle to quarternion
	tf::Quaternion quaternion_begin;
	//tf::Quaternion quaternion = transform.getRotation(); 
	quaternion_begin = tf::createQuaternionFromYaw(theta_begin);
	geometry_msgs::Quaternion qMsg_begin;
	tf::quaternionTFToMsg(quaternion_begin, qMsg_begin);

	int begin_row = grid_x, begin_colum = grid_y;

	pair<int, int> source(begin_row, begin_colum);
	start = clock();


	vector<int>path_v4;


	adj_list.resize(numPoints);//o(1)






	path_v4.clear();
	//o(n^2)
	for (int j = 0; j < matrix.size(); j++) {


		for (int i = 0; i < matrix.size(); i++) {
			if (i + 1 != matrix.size()) {
				if (matrix[i + 1][j].second == 1 && matrix[i][j].second != -1) {
					adj_list[matrix[i][j].first].push_back(matrix[i + 1][j]);

				}
			}

			if (i - 1 >= 0) {
				if (matrix[i - 1][j].second == 1 && matrix[i][j].second != -1) {
					adj_list[matrix[i][j].first].push_back(matrix[i - 1][j]);

				}
			}


			if (j + 1 != matrix[i].size()) {

				if (matrix[i][j + 1].second == 1 && matrix[i][j].second != -1) {
					adj_list[matrix[i][j].first].push_back(matrix[i][j + 1]);

				}

			}

			if (j - 1 >= 0) {

				if (matrix[i][j - 1].second == 1 && matrix[i][j].second != -1) {
					adj_list[matrix[i][j].first].push_back(matrix[i][j - 1]);

				}

			}




		}
	}

	int it = matrix[begin_row][begin_colum].first;//o(n)







	int next_node = it;

	int flag = 0;

	ros::NodeHandle n2;

	ros::Subscriber maps;

	vector<int>path_2;


	MoveBaseClient ac("move_base", true);

	while (!ac.waitForServer(ros::Duration(5.0))) {
		ROS_INFO("Waiting for the move_base action server");
	}

	move_base_msgs::MoveBaseGoal goal;

	goal.target_pose.header.frame_id = "map";



	double x, pos_x, pos_y, y;
	int flag_count = 0;
	int current_point = next_node;
	nav_msgs::GetMap map_srv;
	vector<vector<int>> current_grid;
	int row, col;
	row = source.first;
	col = source.second;
	vector<int> list_of_not_succeeded(numPoints, 0);
	int count_nonv = 0;
	vector<int> last_unstacknode;
	vector<int> last_unstacknode_check(numPoints, 0);
	last_unstacknode.push_back(next_node);

	int old_node;
	int old_node2;
	int first_time = 0;
	vector<int> threepoints;
	int timeout = 20;
	while (find(check.begin(), check.end(), 0) != check.end()) {

		ros::ServiceClient map_fetcher = nh.serviceClient<nav_msgs::GetMap>("dynamic_map");

		if (map_fetcher.call(map_srv))
		{
			current_map = map_srv.response.map;

		}




		if (!check_point(m[next_node].first, m[next_node].second, obscall_rang)) {
			check[next_node] = 2;
			ROS_INFO("check value %d", check[next_node]);


		}
		//if(flag_count>=5)
		// update(next_node,current_map,15);		

		if (flag_count != 0 && matrix[m[next_node].first][m[next_node].second].second == 1 && check[next_node] == 0) {



			if (threepoints.size() != 3) {
				threepoints.push_back(next_node);
			}
			if (threepoints.size() == 3) {
				if (!are_they_on_same_line(m[threepoints[0]], m[threepoints[1]], m[threepoints[2]])) {
					list_of_not_succeeded[threepoints[1]] = 2;

				}
				old_node = threepoints[1];
				threepoints.clear();

				threepoints.push_back(old_node);
				threepoints.push_back(next_node);
			}

			listener.waitForTransform("/map", "/base_link", ros::Time(), ros::Duration(100));
			listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
			current_x = transform.getOrigin().x();
			current_y = transform.getOrigin().y();
			pos_x = ((m[next_node].first * result.info.resolution) + result.info.origin.position.x);
			pos_y = ((m[next_node].second * result.info.resolution) + result.info.origin.position.y);
			x = pos_x - current_x;
			y = pos_y - current_y;
			/*calculate angle*/
			goal.target_pose.pose.position.x = current_x;
			goal.target_pose.pose.position.y = current_y;
			double theta = atan2(y, x);
			// convert angle to quarternion
			tf::Quaternion quaternion;
			//tf::Quaternion quaternion = transform.getRotation(); 
			quaternion = tf::createQuaternionFromYaw(theta);
			geometry_msgs::Quaternion qMsg;
			tf::quaternionTFToMsg(quaternion, qMsg);
			// set quarternion to goal
			goal.target_pose.pose.orientation = qMsg;
			goal.target_pose.header.stamp = ros::Time::now();
			//ROS_INFO("Sending goal");
			//ROS_INFO("x=%f y=%f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
			//ac.sendGoal(goal);
			//ac.waitForResult(ros::Duration(15));
			goal.target_pose.header.stamp = ros::Time::now();
			goal.target_pose.pose.position.x = pos_x;
			goal.target_pose.pose.position.y = pos_y;
			//printf("%d", matrix[m[next_node].first][m[next_node].second]);
			ROS_INFO("Sending goal");
			ROS_INFO(" goal x=%f y=%f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
			ac.sendGoal(goal);
			ac.waitForResult(ros::Duration(timeout));


		}

		if (ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED && flag_count != 0 && check_point(m[next_node].first, m[next_node].second, obscall_rang)) {
			ROS_INFO(" not SUCCEEDED goal");
			count_nonv++;
			list_of_not_succeeded[next_node] = 1;

		}if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED && flag_count != 0 && check_point(m[next_node].first, m[next_node].second, obscall_rang)) {
			count_nonv = 0;
			if (list_of_not_succeeded[next_node] != 2)
				list_of_not_succeeded[next_node] = 0;
			ROS_INFO("SUCCEEDED goal");
			last_unstacknode_check[next_node] = 1;
			last_unstacknode.push_back(next_node);

		}
		if (count_nonv == 2) {
			count_nonv = 0;
			next_node = last_unstacknode[last_unstacknode.size() - 2];
			check[next_node] = 0;
			continue;

		}
		cover(next_node, cover_rang, 1);
		flag_count += 1;




		int x_c = m[next_node].first;
		int y_c = m[next_node].second;
		int x_o = m[next_node].first;
		int y_o = m[next_node].second;
		int node_num_c;
		int count_c = 0;
		bool found = false;

		while (!found && find(check.begin(), check.end(), 0) != check.end()) {
			node_num_c = next_node;

			if (check[node_num_c] != 0) {
				while (x_c + 1 < matrix.size() && check[node_num_c] == 1 && count_c < 15) {
					x_c++;
					count_c++;
					node_num_c = matrix[x_c][y_c].first;
					if (!check_point(x_c, y_c, obscall_rang)) {
						check[node_num_c] = 2;
					}
				}
				count_c = 0;
				if (check[node_num_c] == 0) {
					found = true;
				}
				else {
					x_c = x_o;
					y_c = y_o;
					node_num_c = matrix[x_c][y_c].first;
				}
				while (x_c - 1 >= 0 && check[node_num_c] == 1 && found == false && count_c < 15) {
					x_c--;
					count_c++;
					node_num_c = matrix[x_c][y_c].first;
					if (!check_point(x_c, y_c, obscall_rang)) {
						check[node_num_c] = 2;
					}

				}
				count_c = 0;
				if (check[node_num_c] == 0) {
					found = true;
				}
				else {
					x_c = x_o;
					y_c = y_o;
					node_num_c = matrix[x_c][y_c].first;
				}
				while (y_c - 1 >= 0 && check[node_num_c] == 1 && found == false && count_c < 15) {
					count_c++;
					y_c--;
					node_num_c = matrix[x_c][y_c].first;
					if (!check_point(x_c, y_c, obscall_rang)) {
						check[node_num_c] = 2;
					}

				}
				count_c = 0;
				if (check[node_num_c] == 0) {
					found = true;
				}
				else {
					x_c = x_o;
					y_c = y_o;
					node_num_c = matrix[x_c][y_c].first;
				}
				while (y_c + 1 < matrix.size() && check[node_num_c] == 1 && found == false && count_c < 15) {
					y_c++;
					count_c++;
					node_num_c = matrix[x_c][y_c].first;
					if (!check_point(x_c, y_c, obscall_rang)) {
						check[node_num_c] = 2;
					}
				}
				count_c = 0;
				if (check[node_num_c] == 0) {
					found = true;
				}
				else {
					x_c = x_o;
					y_c = y_o;
					node_num_c = matrix[x_c][y_c].first;
				}


				if (found == false) {

					pair<int, int> rs;
					vector<int> longPath;
					ROS_INFO("dikstra is used");
					rs = Nearest(check, next_node);
					node_num_c = rs.first;
					found = true;
					timeout = rs.second / 2;
					if (timeout == 0)
						timeout = 20;
					ROS_INFO("dikstra path size %d", timeout);
					if (node_num_c == -1) {
						found = false;
						timeout = 20;

					}
				}
				if (node_num_c == INF) {

					next_node++;
					check[next_node] = 1;
				}
				else
					next_node = node_num_c;


			}

		}
		int nonv_node = next_node;
		while (find(list_of_not_succeeded.begin(), list_of_not_succeeded.end(), 1) != list_of_not_succeeded.end() || find(list_of_not_succeeded.begin(), list_of_not_succeeded.end(), 2) != list_of_not_succeeded.end()) {



			nonv_node = Nearest2(list_of_not_succeeded, nonv_node);
			if (check[nonv_node] == 1 || check[nonv_node] == 2) {
				list_of_not_succeeded[nonv_node] = 0;
				ROS_INFO("continue");
				continue;
			}




			if (nonv_node == INF) {

				nonv_node = find(list_of_not_succeeded);

			}
			list_of_not_succeeded[nonv_node] = 0;
			ROS_INFO("list_of_not_succeeded value %d", list_of_not_succeeded[nonv_node]);
			listener.waitForTransform("/map", "/base_link", ros::Time(), ros::Duration(100));
			listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);

			current_x = transform.getOrigin().x();
			current_y = transform.getOrigin().y();
			pos_x = ((m[nonv_node].first * result.info.resolution) + result.info.origin.position.x);
			pos_y = ((m[nonv_node].second * result.info.resolution) + result.info.origin.position.y);

			x = pos_x - current_x;
			y = pos_y - current_y;




			/*calculate angle*/


			goal.target_pose.pose.position.x = current_x;
			goal.target_pose.pose.position.y = current_y;




			double theta = atan2(y, x);
			// convert angle to quarternion
			tf::Quaternion quaternion;
			//tf::Quaternion quaternion = transform.getRotation(); 
			quaternion = tf::createQuaternionFromYaw(theta);
			geometry_msgs::Quaternion qMsg;
			tf::quaternionTFToMsg(quaternion, qMsg);
			// set quarternion to goal
			goal.target_pose.pose.orientation = qMsg;
			goal.target_pose.header.stamp = ros::Time::now();

			ROS_INFO("Sending goal");
			ROS_INFO("x=%f y=%f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
			//ac.sendGoal(goal);
			//ac.waitForResult(ros::Duration(30));
			goal.target_pose.header.stamp = ros::Time::now();
			goal.target_pose.pose.position.x = pos_x;
			goal.target_pose.pose.position.y = pos_y;


			ROS_INFO("Sending goal");
			ROS_INFO(" goal x=%f y=%f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
			ac.sendGoal(goal);

			ac.waitForResult(ros::Duration(30));





		}

}
		goal.target_pose.pose.position.x = map_x;
		goal.target_pose.pose.position.y = map_y;
		goal.target_pose.pose.orientation = qMsg_begin;
		goal.target_pose.header.stamp = ros::Time::now();


		ac.sendGoal(goal);
		ac.waitForResult();
		//printf("finish");
		//save_map(nh,argv[1]);
		ros::spinOnce();

		return 0;

	}





	void save_map(ros::NodeHandle nh, char* path) {
		/* ros::ServiceClient map_fetcher=nh.serviceClient<nav_msgs::GetMap>("dynamic_map");
		 nav_msgs::GetMap map_srv;
	  ROS_INFO("enter");
		if(map_fetcher.call(map_srv))
		   {

				   if (is_map_changed(map_srv.response.map,path))
		  {
			  ROS_INFO("map change");
			  rosbag::Bag map_storage;
			  map_storage.open(map_path, rosbag::bagmode::Write);
			  map_storage.write<nav_msgs::OccupancyGrid>("map", ros::Time::now(), map_srv.response.map);
			  map_storage.close();
			  return;
		  }
	  ROS_INFO("map change has n't chang");


		   }

	  */




	}







	void readMap(char* path)
	{
		nav_msgs::OccupancyGrid   map;

		rosbag::Bag map_reader;
		nav_msgs::OccupancyGridConstPtr  i;
		map_reader.open(path);//absolute path to bag file
		for (rosbag::MessageInstance const m : rosbag::View(map_reader))
		{
			i = m.instantiate<nav_msgs::OccupancyGrid>();
			if (i != nullptr)
				map = *i;
		}
		map_reader.close();
		result = map;
		ROS_INFO("Received a %d X %d map @ %.3f m/px\n", map.info.width, map.info.height, map.info.resolution);
		int rows = map.info.height;
		int cols = map.info.width;

		grid.resize(rows);

		for (int i = 0; i < rows; i++)
		{
			grid[i].resize(cols);
		}
		int currCell = 0;
		for (int i = 0; i < rows; i++)
		{
			for (int j = 0; j < cols; j++) {
				if (map.data[currCell] == 0) { // unoccupied cell
					grid[currCell % rows][currCell / rows] = 0;

				}
				else {
					grid[currCell % rows][currCell / rows] = 1; // occupied (100) or unknown cell (-1)

				}
				currCell++;
			}

		}

	}
