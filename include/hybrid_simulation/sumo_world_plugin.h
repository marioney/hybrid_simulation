#ifndef SUMO_WORLD_PLUGIN_H
#define SUMO_WORLD_PLUGIN_H

#include <gazebo-8/gazebo/common/Plugin.hh>
#include <gazebo-8/gazebo/physics/physics.hh>
#include <gazebo-8/gazebo/util/system.hh>
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>


namespace gazebo
{

class SumoWorldPlugin : public WorldPlugin
{
public:
    SumoWorldPlugin();
    /// \brief Destructor
    ~SumoWorldPlugin();
    /// \brief Load the world plugin.
    /// \param[in] _model Pointer to the parent model.
    /// \param[in] _sdf Pointer to the plugin's SDF elements.
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);
    /// \brief Callback for World Update events.
    private: void OnUpdate();

    /// \brief World pointer.
    private: physics::WorldPtr world_;

    /// \brief Connection to World Update events.
    private: event::ConnectionPtr updateConnection_;

    std::string ego_vehicle_model_name_;

    boost::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    boost::shared_ptr<tf2_ros::Buffer> tf_buffer_;


};
}

#endif // SUMO_WORLD_PLUGIN_H
