
#ifndef __coverage__directGo__
#define __coverage__directGo__

#include <stdio.h>
#include "coverage.h"


vector<Node> directGoPath(vector<vector<int>> &costmap, vector <vector<int> > &grid, vector<Node> &path, Node &curPos, Node toGo);
bool directGo(Node curPos, Node start, Node toGo, vector<vector<int> > &costmap, vector<Node> &directpath);

#endif /* defined(__coverage__directGo__) */