#include "animator.h"
#include <cassert>

namespace USTC_CG::character_animation {

using namespace pxr;

Joint::Joint(int idx, string name, int parent_idx, const GfMatrix4f& bind_transform) 
: idx_(idx), name_(name), parent_idx_(parent_idx), bind_transform_(bind_transform)
{
}

void Joint::compute_world_transform()
{
    // ---------- (HW TODO) Compute world space trasform of this joint -----------------
	if (parent_!=nullptr) {
		world_transform_=local_transform_*parent_->world_transform_;
	}else {
		world_transform_=local_transform_;
	}

	std::cout << "Joint " << name_ << " (idx=" << idx_ << ") Transform:\n";
	std::cout << "Local Transform:\n" << local_transform_ << "\n";
	std::cout << "World Transform:\n" << world_transform_ << "\n";
    // --------------------------------------------------------------------------------
}

void JointTree::compute_world_transforms_for_each_joint()
{
    // ----------- (HW_TODO) Traverse all joint and compute its world space transform ---
	// Call compute_world_transform for each joint
	std::function<void(shared_ptr<Joint>)>traverse=[&](shared_ptr<Joint> joint) {
		joint->compute_world_transform();
		std::cout << "Joint " << joint->name_
		  << " (Parent: " << (joint->parent_ ? joint->parent_->name_ : "None") << ")\n";
		for (auto& child:joint->children_) {
			traverse(child);
		}
	};

	if (root_) {
		traverse(root_);
	}
    // ---------------------------------------------
}

void JointTree::add_joint(int idx, std::string name, int parent_idx, const GfMatrix4f& bind_transform)
{
    auto joint = make_shared<Joint>(idx, name, parent_idx, bind_transform);
    joints_.push_back(joint);
    if (parent_idx < 0) {
        root_ = joint;
    }
    else {
        joints_[parent_idx]->children_.push_back(joint);

        if (parent_idx < joints_.size())
            joint->parent_ = joints_[parent_idx];
        else {
            std::cout << "[add_joint_error] parent_idx out of range" << std::endl;
            exit(1);
        }
    }
}

void JointTree::update_joint_local_transform(const VtArray<GfMatrix4f>& new_local_transforms)
{
    assert(new_local_transforms.size() == joints_.size());

    for (int i = 0; i < joints_.size(); ++i) {
		joints_[i]->local_transform_ = new_local_transforms[i];
	}
}

void JointTree::print()
{
	for (auto joint_ptr : joints_) {
		std::cout << "Joint idx: " << joint_ptr->idx_ << " name: " << joint_ptr->name_ << " parent_idx: " << joint_ptr->parent_idx_ << std::endl;
	}
}


Animator::Animator(const shared_ptr<MeshComponent> mesh, const shared_ptr<SkelComponent> skel)
	: mesh_(mesh),
      skel_(skel)
{
    auto joint_order = skel_->jointOrder;
    auto topology = skel_->topology;
    for (size_t i = 0; i < joint_order.size(); ++i) {
	    SdfPath jointPath(joint_order[i]);

        string joint_name = jointPath.GetName();
        int parent_idx = topology.GetParent(i);

		joint_tree_.add_joint(i, joint_name, parent_idx, GfMatrix4f(skel->bindTransforms[i]));
	}

	joint_tree_.print();
}

void Animator::step(const shared_ptr<SkelComponent> skel)
{
	joint_tree_.update_joint_local_transform(skel->localTransforms);

    joint_tree_.compute_world_transforms_for_each_joint();

	update_mesh_vertices();
}

void Animator::update_mesh_vertices()
{
	// ----------- (HW_TODO) Update mesh vertices according to the current joint transforms ----
	// 1. get skel_->jointIndices and skel_->jointWeight;
	const auto& jointIndices=skel_->jointIndices;
	const auto& jointWeights=skel_->jointWeight;

	VtArray<GfVec3f> vertices=mesh_->get_vertices();
	int n=jointIndices.size()/vertices.size();


	// 2. For each vertex, compute the new position by transforming the rest position with the joint transforms
	for (int vertex_idx=0;vertex_idx<vertices.size();++vertex_idx) {
		GfVec3f original_pos=vertices[vertex_idx];
		GfVec3f deformed_pos(0.0f);

		for (int i=0;i<n;++i) {
			int joint_idx = jointIndices[vertex_idx*n+i];
			float weight=jointWeights[vertex_idx*n+i];
			if (weight<=0.0f||joint_idx<0||joint_idx>=joint_tree_.get_joints().size()) {
				continue;
			}

			auto joint=joint_tree_.get_joint(joint_idx);
			GfMatrix4f bind_inverse=joint->get_bind_transform().GetInverse();
			GfMatrix4f M_i=(joint->get_world_transform().GetTranspose())*(bind_inverse.GetTranspose());
			M_i=M_i.GetTranspose();


			if (vertex_idx == 0 && i == 0) { // 仅输出第一个顶点的第一个骨骼
				std::cout << "\n--- Debug for Vertex 0, Joint " << joint->name_ << " ---\n";
				std::cout << "Bind Transform (Original):\n" << joint->get_bind_transform() << "\n";
				std::cout << "Bind Inverse:\n" << bind_inverse << "\n";
				std::cout << "World Transform:\n" << joint->get_world_transform() << "\n";
				std::cout << "M_i (World * BindInverse):\n" << M_i << "\n";
				std::cout << "Original Position: " << original_pos << "\n";
				std::cout << "Transformed Position: " << M_i.Transform(original_pos) << "\n";
			}

			GfVec3f transformed_pos=M_i.Transform(original_pos);
			deformed_pos+=transformed_pos*weight;
		}
		vertices[vertex_idx]=deformed_pos;
	}

	// 2. Update the vertex position in the mesh
	mesh_->set_vertices(vertices);
	// --------------------------------------------------------------------------------
}

}  // namespace USTC_CG::character_animation