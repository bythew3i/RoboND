#include <gazebo/gazebo.hh>

namespace gazebo {
    class WorldPluginMyRobot : public WorldPlugin {
      public:
        // Constructor
        WorldPluginMyRobot() : WorldPlugin() {
            printf("Welcome to Jevin's World!\n");
        }
        
        // Mandatory
        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) {

        }
    
    };

    GZ_REGISTER_WORLD_PLUGIN(WorldPluginMyRobot)
}