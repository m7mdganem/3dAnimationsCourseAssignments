#pragma once

#include <Eigen/Core>
#include <igl/decimate.h> // for the min_heap
#include <igl/collapse_edge.h>
#include "circulation.h"
#include "edges_cost_computation.h"
#include <iostream>
#include <iomanip>

bool collapse_edge_improved(
	Eigen::MatrixXd& V,
	Eigen::MatrixXi& F,
	Eigen::MatrixXi& E,
	Eigen::VectorXi& EMAP,
	Eigen::MatrixXi& EF,
	Eigen::MatrixXi& EI,
	igl::min_heap<std::tuple<double, int, int>>& Q,
	Eigen::MatrixXd& C,
	std::vector <Eigen::Matrix4d>& VQ,
	std::vector<double>& EC
);