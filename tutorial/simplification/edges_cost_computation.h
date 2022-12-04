#pragma once

#include <Eigen/Core>
#include <igl/decimate.h> // for the min_heap
#include <Eigen/LU>

void edge_cost_computation(
	int edge_index									,/* Edge index */
	Eigen::MatrixXd& V								,/* Vreticies */
	Eigen::MatrixXi& E								,/* Edges */
	Eigen::MatrixXd& C								,/* New positions */
	igl::min_heap<std::tuple<double, int, int>>& Q	,/* Queue */
	std::vector <Eigen::Matrix4d>& VQ				,/* Vreticies to matricies */
	std::vector<double>& EC							/* Edges costs */
	);