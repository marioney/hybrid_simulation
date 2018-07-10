#include <hybrid_simulation/sumo_world_plugin.h>

namespace gazebo
{
    SumoWorldPlugin::SumoWorldPlugin():
        WorldPlugin()
{

    }

    SumoWorldPlugin::~SumoWorldPlugin()
    {

    }

    void SumoWorldPlugin::Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {


        this->world_ = _world;

        if(_sdf->HasElement("ego_vehicle_model_name"))
        {
            ego_vehicle_model_name_ = _sdf->Get<std::string>("ego_vehicle_model_name");
        }
        else
        {
            ROS_WARN("ego_vehicle_model_name not found, setting default zoe");
            ego_vehicle_model_name_ = "zoe";
        }

        // Create the TF listener for the desired position of the link
        tf_buffer_.reset(new tf2_ros::Buffer());
        tf_listener_.reset(new tf2_ros::TransformListener(*tf_buffer_));

        this->updateConnection_ = event::Events::ConnectWorldUpdateBegin(
                std::bind(&SumoWorldPlugin::OnUpdate, this));

        ROS_INFO("Sumo World Plugin loaded");

    }

    void SumoWorldPlugin::OnUpdate()
    {
        // Get TF transform relative to the /world link
        geometry_msgs::TransformStamped base_link_new_tform;

        std::string model_name;

        for(unsigned int n_model = 0; n_model < world_->ModelCount(); n_model++)
        {
            physics::ModelPtr  tmp_model;
            tmp_model = world_->ModelByIndex(n_model);
            std::string frame_id;
            frame_id = tmp_model->GetName();
            if (frame_id == ego_vehicle_model_name_ )
            {
                continue;
            }
            if (frame_id == "ground")
            {
                continue;
            }

            try
            {
                base_link_new_tform = tf_buffer_->lookupTransform("world", frame_id, ros::Time(0));
            }
            catch (tf2::TransformException ex)
            {
                ROS_ERROR("vehicle: %s - %s", frame_id.c_str(), ex.what());
                continue;
            }
            //
            //    const geometry_msgs::Vector3 &p = base_link_new_tform.transform.translation;
            //    const geometry_msgs::Quaternion &q = base_link_new_tform.transform.rotation;
            //    ROS_INFO(" %s pose %.2f - %.2f ", frame_id_.c_str(),
            //             base_link_new_tform.transform.translation.x, base_link_new_tform.transform.translation.y);



            // Convert TF transform to Gazebo Pose
            ignition::math::Pose3d new_pose;
            new_pose.Pos().Set(base_link_new_tform.transform.translation.x,
                               base_link_new_tform.transform.translation.y,
                               base_link_new_tform.transform.translation.z);
            new_pose.Rot().Set(base_link_new_tform.transform.rotation.w,
                               base_link_new_tform.transform.rotation.x,
                               base_link_new_tform.transform.rotation.y,
                               base_link_new_tform.transform.rotation.z);

            //    ROS_INFO(" %s pose %.2f - %.2f ", frame_id_.c_str(),
            //             new_pose.Pos().X(), new_pose.Pos().Y());

            //    ROS_INFO("Pre set %s", frame_id_.c_str() );
            try
            {
                tmp_model->SetWorldPose(new_pose);
            }
            catch(gazebo::common::Exception gz_ex)
            {
                ROS_ERROR("Error setting pose %s - %s", frame_id.c_str(), gz_ex.GetErrorStr().c_str());
            }
        }
        //    ROS_INFO("Post set");
        //    ignition::math::Pose3d old_pose = model_->WorldPose();
        //    ROS_INFO(" %s pose %.2f - %.2f ", frame_id_.c_str(),
        //             old_pose.Pos().X(), old_pose.Pos().Y());
    }
    GZ_REGISTER_WORLD_PLUGIN(SumoWorldPlugin)
}
