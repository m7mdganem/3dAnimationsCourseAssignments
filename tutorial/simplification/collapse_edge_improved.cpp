#include "collapse_edge_improved.h"

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
	std::vector<double>& EC)
{
	// In this procedure, we do not delete tuples from the heap, instead we mark their cost in the
	// EC vector as NAN, and when we pop a nan-cost-edge we throw it and continue with another one.
	// It is something like "lazy-delete"

	if (Q.empty())
	{
		return false;
	}

	std::tuple<double, int, int> p;
	double cost;
	int e;
	int first_vertex;
	int second_vertex;
	while (true)
	{
		p = Q.top();
		cost = std::get<0>(p);
		e = std::get<1>(p);
		first_vertex = E(e, 0);
		second_vertex = E(e, 1);
		Q.pop();
		if (cost == std::numeric_limits<double>::infinity())
		{
			// min cost edge is infinite cost
			return false;
		}
		else if (cost == EC[e])
		{
			// Delete edge
			EC[e] = std::numeric_limits<double>::quiet_NaN();
			break;
		}
		// Else, it is either a deleted edge (and EC[e] = Nan), or the edge cost was updated and 
		// the tuple that we took from the heap is not updated and there is another tuple for the
		// same edgde in the heap which is updated.
	}

	// Get faces and vertices that will be affected from the collapse
	std::vector<int> Nsf, Nsv;
	igl::circulation(e, true, F, EMAP, EF, EI,Nsv, Nsf);
	std::vector<int> Ndf, Ndv;
	igl::circulation(e, false, F, EMAP, EF, EI,Ndv, Ndf);

	bool something_collapsed = false;
	int e1, e2, f1, f2;
	if (igl::collapse_edge(e, C.row(e), Nsv, Nsf, Ndv, Ndf, V, F, E, EMAP, EF, EI, e1, e2, f1, f2))
	{
		Nsf.insert(Nsf.begin(), Ndf.begin(), Ndf.end());

		something_collapsed = true;

		// Delete edges
		EC[e1] = std::numeric_limits<double>::quiet_NaN();
		EC[e2] = std::numeric_limits<double>::quiet_NaN();

		// The collapse edge does not delete vertices actually, instead it assigns them
		// to point to the same new vertex.
		// Thus we update their Q matrices with the new vertex Q matrix.
		auto new_q = VQ[first_vertex] + VQ[second_vertex];
		VQ[first_vertex] = new_q;
		VQ[second_vertex] = new_q;

		for (auto n : Nsf)
		{
			if (F(n, 0) != IGL_COLLAPSE_EDGE_NULL ||
				F(n, 1) != IGL_COLLAPSE_EDGE_NULL ||
				F(n, 2) != IGL_COLLAPSE_EDGE_NULL)
			{
				for (int v = 0; v < 3; v++)
				{
					// Get edge id
					const int ei = EMAP(v * F.rows() + n);

					// Calculate new cost and vertex for the edge
					// Update Q, C, EC as well
					edge_cost_computation(ei, V, E, C, Q, VQ, EC);
				}
			}
		}

		// The follwing lines prints the required prints with the following format:
		// edge:  22621            |       cost:    2.39005e-06            |       new v position: ( -0.807841,  0.0303555,  -0.163103)
		// edge:  24790            |       cost:    2.39925e-06            |       new v position: ( -0.754239,   0.240327,  -0.177346)
		// edge:   4228            |       cost:    2.40153e-06            |       new v position: ( -0.493257,   0.285505,  -0.132906)
		std::cout << std::setw(6) << "edge: " << std::setfill(' ') << std::setw(6) << e << std::setfill(' ')
			<< std::setw(8) << "\t\t|\tcost: " << std::setfill(' ') << std::setw(14) << cost << std::setfill(' ') 
			<< std::setw(18) << "\t\t|\tnew v position: " << std::setfill(' ') 
			<< "(" << std::setw(10) << C.row(e)[0] << std::setfill(' ')
			<< ", " << std::setw(10) << C.row(e)[1] << std::setfill(' ')
			<< ", " << std::setw(10) << C.row(e)[2] << std::setfill(' ') << ")" << std::endl;
	}
	else
	{
		// If the edge can not be collapsed, re-insert it to the heap
		// with infinity cost
		Q.emplace(std::numeric_limits<double>::infinity(), e, 0);
	}

	return something_collapsed;
}