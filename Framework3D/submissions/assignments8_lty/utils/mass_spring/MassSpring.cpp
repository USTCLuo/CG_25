#include "MassSpring.h"
#include <iostream>

namespace USTC_CG::mass_spring {
MassSpring::MassSpring(const Eigen::MatrixXd& X, const EdgeSet& E)
{
    this->X = this->init_X = X;
    this->vel = Eigen::MatrixXd::Zero(X.rows(), X.cols());
    this->E = E;

    std::cout << "number of edges: " << E.size() << std::endl;
    std::cout << "init mass spring" << std::endl;

    // Compute the rest pose edge length
    for (const auto& e : E) {
        Eigen::Vector3d x0 = X.row(e.first);
        Eigen::Vector3d x1 = X.row(e.second);
        this->E_rest_length.push_back((x0 - x1).norm());
    }

    // Initialize the mask for Dirichlet boundary condition
    dirichlet_bc_mask.resize(X.rows(), false);

    // (HW_TODO) Fix two vertices, feel free to modify this 
    unsigned n_fix = sqrt(X.rows());  // Here we assume the cloth is square
    dirichlet_bc_mask[0] = true;
    dirichlet_bc_mask[n_fix - 1] = true;
}

void MassSpring::step()
{
    Eigen::Vector3d acceleration_ext = gravity + wind_ext_acc;

    unsigned n_vertices = X.rows();

    // The reason to not use 1.0 as mass per vertex: the cloth gets heavier as we increase the resolution
    double mass_per_vertex =
        mass / n_vertices; 

    //----------------------------------------------------
    // (HW Optional) Bonus part: Sphere collision
    Eigen::MatrixXd acceleration_collision =
        getSphereCollisionForce(sphere_center.cast<double>(), sphere_radius);
    //----------------------------------------------------

    if (time_integrator == IMPLICIT_EULER) {
        // Implicit Euler
        TIC(step)

        // (HW TODO) 
        auto H_elastic = computeHessianSparse(stiffness);  // size = [nx3, nx3]
        Eigen::SparseMatrix<double> M(n_vertices*3,n_vertices*3);
        std::vector<Eigen::Triplet<double>> triplets_M;
        for (unsigned i=0;i<n_vertices;++i) {
            for (int j=0;j<3;++j) {
                triplets_M.emplace_back(3*i+j,3*i+j,mass_per_vertex);
            }
        }
        M.setFromTriplets(triplets_M.begin(),triplets_M.end());
        M.makeCompressed();
        // compute Y 
        Eigen::MatrixXd Y = X+h*vel+h*h*M.diagonal().asDiagonal().inverse()*(acceleration_ext.replicate(n_vertices,1));
        Eigen::MatrixXd Y_flatten=flatten(Y);

        Eigen::MatrixXd grad_E=computeGrad(stiffness);
        Eigen::MatrixXd grad_E_flatten=flatten(grad_E);
        Eigen::MatrixXd grad_g =1.0/(h*h)*M*(flatten(X)-Y_flatten)+grad_E_flatten;

        // Solve Newton's search direction with linear solver
        Eigen::SparseMatrix<double> grad2_g=1.0/(h*h)*M+H_elastic;
        Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
        solver.compute(grad2_g);
        if (solver.info()!=Eigen::Success) {
            std::cerr<<"Decomposition failed"<<std::endl;
            return;
        }
        Eigen::MatrixXd delta_X_flatten=solver.solve(-grad_g);
        if (solver.info()!=Eigen::Success) {
            std::cerr<<"Solving failed"<<std::endl;
            return ;
        }
        Eigen::MatrixXd delta_X=unflatten(delta_X_flatten);

        
        // update X and vel
        X+=delta_X;
        vel=delta_X/h;
        for (size_t i=0;i<dirichlet_bc_mask.size();++i) {
            if (dirichlet_bc_mask[i]) {
                vel.row(i).setZero();
                X.row(i)=init_X.row(i);
            }
        }

        TOC(step)
    }
    else if (time_integrator == SEMI_IMPLICIT_EULER) {

        // Semi-implicit Euler
        Eigen::MatrixXd acceleration = -computeGrad(stiffness) / mass_per_vertex;
        acceleration.rowwise() += acceleration_ext.transpose();

        // -----------------------------------------------
        // (HW Optional)
        if (enable_sphere_collision) {
            acceleration += acceleration_collision;
        }
        // -----------------------------------------------

        // (HW TODO): Implement semi-implicit Euler time integration
        vel+=h*acceleration;
        X+=h*vel;

        vel *= damping;

        for (size_t i=0;i<dirichlet_bc_mask.size();++i) {
                if (dirichlet_bc_mask[i]) {
                    vel.row(i).setZero();
                }
        }
        // Update X and vel 
        
    }
    else {
        std::cerr << "Unknown time integrator!" << std::endl;
        return;
    }
}

// There are different types of mass spring energy:
// For this homework we will adopt Prof. Huamin Wang's energy definition introduced in GAMES103
// course Lecture 2 E = 0.5 * stiffness * sum_{i=1}^{n} (||x_i - x_j|| - l)^2 There exist other
// types of energy definition, e.g., Prof. Minchen Li's energy definition
// https://www.cs.cmu.edu/~15769-f23/lec/3_Mass_Spring_Systems.pdf
double MassSpring::computeEnergy(double stiffness)
{
    double sum = 0.;
    unsigned i = 0;
    for (const auto& e : E) {
        auto diff = X.row(e.first) - X.row(e.second);
        auto l = E_rest_length[i];
        sum += 0.5 * stiffness * std::pow((diff.norm() - l), 2);
        i++;
    }
    return sum;
}

Eigen::MatrixXd MassSpring::computeGrad(double stiffness)
{
    Eigen::MatrixXd g = Eigen::MatrixXd::Zero(X.rows(), X.cols());
    unsigned i = 0;
    for (const auto& e : E) {
        // --------------------------------------------------
        // (HW TODO): Implement the gradient computation
        Eigen::Vector3d x0 = X.row(e.first);
        Eigen::Vector3d x1 = X.row(e.second);
        Eigen::Vector3d xi=x0-x1;
        double Li=E_rest_length[i];
        double norm_xi=xi.norm();
        if (norm_xi<1e-10) {
            continue;
        }
        Eigen::Vector3d grad=stiffness*(norm_xi-Li)*xi/norm_xi;

        g.row(e.first)+=grad;
        g.row(e.second)-=grad;
        // --------------------------------------------------
        i++;
    }
    return g;
}

Eigen::SparseMatrix<double> MassSpring::computeHessianSparse(double stiffness)
{
    unsigned n_vertices = X.rows();
    Eigen::SparseMatrix<double> H(n_vertices * 3, n_vertices * 3);
    std::vector<Eigen::Triplet<double>> triplets;

    unsigned i = 0;
    auto k = stiffness;
    const auto I = Eigen::MatrixXd::Identity(3, 3);
    for (const auto& e : E) {
        // --------------------------------------------------
        // (HW TODO): Implement the sparse version Hessian computation
        Eigen::Vector3d x0=X.row(e.first);
        Eigen::Vector3d x1=X.row(e.second);
        Eigen::Vector3d xi=x0-x1;
        double Li=E_rest_length[i];
        double norm_xi=xi.norm();

        Eigen::Matrix3d Hi;
        if (Li>norm_xi) {
            Hi=k*xi*xi.transpose()/(norm_xi*norm_xi);

        }else {
            Hi=k*xi*xi.transpose()/(norm_xi*norm_xi)+k*(1-Li/norm_xi)*(I-xi*xi.transpose()/(norm_xi*norm_xi));

        }
        for (int j=0;j<3;j++) {

            for (int l=0;l<3;++l) {
                triplets.emplace_back(3 * e.first + j, 3 * e.first + l, Hi(j, l));
                triplets.emplace_back(3 * e.first + j, 3 * e.second + l, -Hi(j, l));
                triplets.emplace_back(3 * e.second + j, 3 * e.first + l, -Hi(j, l));
                triplets.emplace_back(3 * e.second + j, 3 * e.second + l, Hi(j, l));

            }
        }
        // Remember to consider fixed points 
        // You can also consider positive definiteness here
       
        // --------------------------------------------------

        i++;
    }
    H.setFromTriplets(triplets.begin(),triplets.end());
    H.makeCompressed();

    //fixed points
    for (size_t i=0;i<dirichlet_bc_mask.size();++i) {
        if (dirichlet_bc_mask[i]) {
            for (int j=0;j<3;++j) {
                int idx=3*i+j;
                H.coeffRef(idx,idx)=1.0;
                for (Eigen::SparseMatrix<double>::InnerIterator it(H,idx);it;++it) {
                    if (it.col()!=idx) {
                        H.coeffRef(idx,it.col())=0.0;
                        H.coeffRef(it.col(),idx)=0.0;
                    }
                }
            }
        }
    }
    return H;
}


bool MassSpring::checkSPD(const Eigen::SparseMatrix<double>& A)
{
    // Eigen::SimplicialLDLT<SparseMatrix_d> ldlt(A);
    // return ldlt.info() == Eigen::Success;
    Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> es(A);
    auto eigen_values = es.eigenvalues();
    return eigen_values.minCoeff() >= 1e-10;
}

void MassSpring::reset()
{
    std::cout << "reset" << std::endl;
    this->X = this->init_X;
    this->vel.setZero();
}

// ----------------------------------------------------------------------------------
// (HW Optional) Bonus part
Eigen::MatrixXd MassSpring::getSphereCollisionForce(Eigen::Vector3d center, double radius)
{
    Eigen::MatrixXd force = Eigen::MatrixXd::Zero(X.rows(), X.cols());
    for (int i = 0; i < X.rows(); i++) {
       // (HW Optional) Implement penalty-based force here 
    }
    return force;
}
// ----------------------------------------------------------------------------------
 
bool MassSpring::set_dirichlet_bc_mask(const std::vector<bool>& mask)
{
	if (mask.size() == X.rows())
	{
		dirichlet_bc_mask = mask;
		return true;
	}
	else
		return false;
}

bool MassSpring::update_dirichlet_bc_vertices(const MatrixXd &control_vertices)
{
   for (int i = 0; i < dirichlet_bc_control_pair.size(); i++)
   {
       int idx = dirichlet_bc_control_pair[i].first;
	   int control_idx = dirichlet_bc_control_pair[i].second;
	   X.row(idx) = control_vertices.row(control_idx);
   }

   return true; 
}

bool MassSpring::init_dirichlet_bc_vertices_control_pair(const MatrixXd &control_vertices,
    const std::vector<bool>& control_mask)
{
    
	if (control_mask.size() != control_vertices.rows())
			return false; 

   // TODO: optimize this part from O(n) to O(1)
   // First, get selected_control_vertices
   std::vector<VectorXd> selected_control_vertices; 
   std::vector<int> selected_control_idx; 
   for (int i = 0; i < control_mask.size(); i++)
   {
       if (control_mask[i])
       {
			selected_control_vertices.push_back(control_vertices.row(i));
            selected_control_idx.push_back(i);
		}
   }

   // Then update mass spring fixed vertices 
   for (int i = 0; i < dirichlet_bc_mask.size(); i++)
   {
       if (dirichlet_bc_mask[i])
       {
           // O(n^2) nearest point search, can be optimized
           // -----------------------------------------
           int nearest_idx = 0;
           double nearst_dist = 1e6; 
           VectorXd X_i = X.row(i);
           for (int j = 0; j < selected_control_vertices.size(); j++)
           {
               double dist = (X_i - selected_control_vertices[j]).norm();
               if (dist < nearst_dist)
               {
				   nearst_dist = dist;
				   nearest_idx = j;
			   }
           }
           //-----------------------------------------
           
		   X.row(i) = selected_control_vertices[nearest_idx];
           dirichlet_bc_control_pair.push_back(std::make_pair(i, selected_control_idx[nearest_idx]));
	   }
   }

   return true; 
}

}  // namespace USTC_CG::node_mass_spring

