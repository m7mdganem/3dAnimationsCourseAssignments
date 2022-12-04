#include "edges_cost_computation.h"

void edge_cost_computation(
	int edge_index,
	Eigen::MatrixXd& V,
	Eigen::MatrixXi& E,
	Eigen::MatrixXd& C,
	igl::min_heap<std::tuple<double, int, int>>& Q,
	std::vector <Eigen::Matrix4d>& VQ,
	std::vector<double>& EC)
{
	if (EC[edge_index] == std::numeric_limits<double>::quiet_NaN())
	{
		// Deleted edge
		return;
	}

	// Calculate the new matrix -> Q = Q[v1] + Q[v2]
	Eigen::Matrix4d q = VQ[E(edge_index, 0)] + VQ[E(edge_index, 1)];

	// If the new matrix Q is invertible then qal
	Eigen::Matrix4d q_inversed = q;
	q_inversed.row(3) = Eigen::Vector4d(0, 0, 0, 1);
	bool is_invertible = q_inversed.determinant() != 0;

	// Calculate vertex position and calculate the cost of the edge
	Eigen::Vector4d new_vertex_position;
	double edge_cost;
	if (is_invertible)
	{
		q_inversed = q_inversed.inverse().eval();
		new_vertex_position = q_inversed * (Eigen::Vector4d(0, 0, 0, 1));
		edge_cost = new_vertex_position.transpose() * q * new_vertex_position;
	}
	else
	{
		auto first_vertex = V.row(E(edge_index, 0));
		Eigen::Vector4d v1_extended;
		v1_extended[0] = first_vertex[0];
		v1_extended[1] = first_vertex[1];
		v1_extended[2] = first_vertex[2];
		v1_extended[3] = 1;
		double v1_cost = v1_extended.transpose() * q * v1_extended;

		auto second_vertex = V.row(E(edge_index, 1));
		Eigen::Vector4d v2_extended;
		v2_extended[0] = second_vertex[0];
		v2_extended[1] = second_vertex[1];
		v2_extended[2] = second_vertex[2];
		v2_extended[3] = 1;
		double v2_cost = v2_extended.transpose() * q * v2_extended;

		auto combined_verticies = (first_vertex + second_vertex) / 2;
		Eigen::Vector4d combined_extended;
		combined_extended[0] = combined_verticies[0];
		combined_extended[1] = combined_verticies[1];
		combined_extended[2] = combined_verticies[2];
		combined_extended[3] = 1;
		double combined_cost = combined_extended.transpose() * q * combined_extended;

		double min_cost = std::min(v1_cost, v2_cost);
		min_cost = std::min(min_cost, combined_cost);

		if (min_cost == v1_cost)
		{
			new_vertex_position = v1_extended;
			edge_cost = v1_cost;
		}
		else if (min_cost == v2_cost)
		{
			new_vertex_position = v2_extended;
			edge_cost = v2_cost;
		}
		else
		{
			new_vertex_position = combined_extended;
			edge_cost = combined_cost;
		}
	}

	Eigen::Vector3d new_position;
	new_position[0] = new_vertex_position[0];
	new_position[1] = new_vertex_position[1];
	new_position[2] = new_vertex_position[2];

	C.row(edge_index) = new_position;
	Q.emplace(edge_cost, edge_index, 0);
	EC[edge_index] = edge_cost;
}