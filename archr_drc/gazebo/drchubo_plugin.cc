#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <iostream>
#include <gazebo/transport/TransportTypes.hh>
#include <gazebo/msgs/MessageTypes.hh>
#include <gazebo/common/Time.hh>
#include <time.h>
#include <math.h>

/* Required Hubo Headers */
#include <hubo.h>

// For Ach
#include <errno.h>
#include <fcntl.h>
//#include <assert.h> //raises an error
#include <unistd.h>
#include <pthread.h>
#include <ctype.h>
#include <stdbool.h>
#include <math.h>
#include <inttypes.h>
#include <ach.h>
#include <string.h>

// ach channels
ach_channel_t chan_hubo_ref;      // hubo-ach

int i = 0;

//define drc controller joints
float posRSP = 0;
/* Create initial structures to read and write from */
struct hubo_ref H_ref;
/* for size check */
size_t fs;

namespace gazebo
{   
  class DrcHuboPlugin : public ModelPlugin
  {

    // Function is called everytime a message is received.
//void cb(gazebo::msgs::Image &_msg)
//void cb(const std::string& _msg)
//void cb(gazebo::msgs::ImageStamped &_msg)
//void cb(ConstWorldStatisticsPtr &_msg)
//void cb(const std::string& _msg)
    public: void cb(ConstWorldStatisticsPtr &_msg)
    {
       gazebo::common::Time simTime  = gazebo::msgs::Convert(_msg->sim_time());
       //size_t size;
       double ttime = simTime.Double();
       //ach_put(&chan_time, _msg->image().data().c_str() , _msg->image().data().size());

    }


    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf) 
    {
      // Open Ach channel
     // Open Ach channel
        /* open ach channel */
      int r = ach_open(&chan_hubo_ref, HUBO_CHAN_REF_NAME , NULL);
      /* Create initial structures to read and write from */
      memset( &H_ref, 0, sizeof(H_ref));

      //assert( ACH_OK == r );

      // Store the pointer to the model
      this->model = _parent;

      // Get then name of the parent model
//      std::string modelName = _sdf->GetParent()->Get<std::string>("name");

      // Get the world name.
//      std::string worldName = _sdf->GetName();
      this->world = physics::get_world("default");


      // Load parameters for this plugin
      if (this->LoadParams(_sdf))
      {
        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            boost::bind(&DrcHuboPlugin::OnUpdate, this));
      }

      // subscribe to thread
//      gazebo::transport::NodePtr node(new gazebo::transport::Node());
//      node->Init();
//      gazebo::transport::SubscriberPtr sub = node->Subscribe("/gazebo/default/world_stats", cb);
    }

    public: bool LoadParams(sdf::ElementPtr _sdf) //assigning joints that are defined .sdf file
    {
      if (this->FindJointByParam(_sdf, this->joint_LSP_,      "LSP") &&
          this->FindJointByParam(_sdf, this->joint_LSR_,      "LSR") &&
          this->FindJointByParam(_sdf, this->joint_LSY_,      "LSY") &&
          this->FindJointByParam(_sdf, this->joint_LEP_,      "LEP") &&
          this->FindJointByParam(_sdf, this->joint_LWP_,      "LWP") &&
          this->FindJointByParam(_sdf, this->joint_LWR_,      "LWR") &&
          this->FindJointByParam(_sdf, this->joint_LWY_,      "LWY") &&

          this->FindJointByParam(_sdf, this->joint_RSP_,      "RSP") &&
          this->FindJointByParam(_sdf, this->joint_RSR_,      "RSR") &&
          this->FindJointByParam(_sdf, this->joint_RSY_,      "RSY") &&
          this->FindJointByParam(_sdf, this->joint_REP_,      "REP") &&
          this->FindJointByParam(_sdf, this->joint_RWP_,      "RWP") &&
          this->FindJointByParam(_sdf, this->joint_RWR_,      "RWR") &&
          this->FindJointByParam(_sdf, this->joint_RWY_,      "RWY") &&

          this->FindJointByParam(_sdf, this->joint_NK1_,      "NK1") &&
          this->FindJointByParam(_sdf, this->joint_NK2_,      "NK2") &&
          this->FindJointByParam(_sdf, this->joint_NK3_,      "NK3") &&

          this->FindJointByParam(_sdf, this->joint_TSY_,      "TSY") &&

          this->FindJointByParam(_sdf, this->joint_LF1_,      "LF1") &&
//          this->FindJointByParam(_sdf, this->joint_LF11_,     "LF11") &&
          this->FindJointByParam(_sdf, this->joint_LF12_,     "LF12") &&
          this->FindJointByParam(_sdf, this->joint_LF13_,     "LF13") &&
//          this->FindJointByParam(_sdf, this->joint_LF2_,      "LF2") &&
          this->FindJointByParam(_sdf, this->joint_LF21_,     "LF21") &&
          this->FindJointByParam(_sdf, this->joint_LF22_,     "LF22") &&
          this->FindJointByParam(_sdf, this->joint_LF23_,     "LF23") &&
//          this->FindJointByParam(_sdf, this->joint_LF3_,      "LF3") &&
          this->FindJointByParam(_sdf, this->joint_LF31_,     "LF31") &&
          this->FindJointByParam(_sdf, this->joint_LF32_,     "LF32") &&
          this->FindJointByParam(_sdf, this->joint_LF33_,     "LF33") &&

          this->FindJointByParam(_sdf, this->joint_LAP_,      "LAP") &&
          this->FindJointByParam(_sdf, this->joint_LAR_,      "LAR") &&
          this->FindJointByParam(_sdf, this->joint_LKP_,      "LKP") &&
          this->FindJointByParam(_sdf, this->joint_LHP_,      "LHP") &&
          this->FindJointByParam(_sdf, this->joint_LHR_,      "LHR") &&
          this->FindJointByParam(_sdf, this->joint_LHY_,      "LHY") &&

          this->FindJointByParam(_sdf, this->joint_RF1_,      "RF1") &&
//          this->FindJointByParam(_sdf, this->joint_RF11_,     "RF11") &&
          this->FindJointByParam(_sdf, this->joint_RF12_,     "RF12") &&
          this->FindJointByParam(_sdf, this->joint_RF13_,     "RF13") &&
          this->FindJointByParam(_sdf, this->joint_RF2_,      "RF2") &&
          this->FindJointByParam(_sdf, this->joint_RF21_,     "RF21") &&
          this->FindJointByParam(_sdf, this->joint_RF22_,     "RF22") &&
          this->FindJointByParam(_sdf, this->joint_RF23_,     "RF23") &&
//          this->FindJointByParam(_sdf, this->joint_RF3_,      "RF3") &&
          this->FindJointByParam(_sdf, this->joint_RF31_,     "RF31") &&
          this->FindJointByParam(_sdf, this->joint_RF32_,     "RF32") &&
          this->FindJointByParam(_sdf, this->joint_RF33_,     "RF33") &&
//          this->FindJointByParam(_sdf, this->joint_RF4_,     "RF4") &&
//          this->FindJointByParam(_sdf, this->joint_RF41_,     "RF41") &&
//          this->FindJointByParam(_sdf, this->joint_RF42_,     "RF42") &&
//          this->FindJointByParam(_sdf, this->joint_RF43_,     "RF43") &&

          this->FindJointByParam(_sdf, this->joint_LAP_,      "LAP") &&
          this->FindJointByParam(_sdf, this->joint_LAR_,      "LAR") &&
          this->FindJointByParam(_sdf, this->joint_LKP_,      "LKP") &&
          this->FindJointByParam(_sdf, this->joint_LHP_,      "LHP") &&
          this->FindJointByParam(_sdf, this->joint_LHR_,      "LHR") &&
          this->FindJointByParam(_sdf, this->joint_LHY_,      "LHY") &&

          this->FindJointByParam(_sdf, this->joint_RAP_,      "RAP") &&
          this->FindJointByParam(_sdf, this->joint_RAR_,      "RAR") &&
          this->FindJointByParam(_sdf, this->joint_RKP_,      "RKP") &&
          this->FindJointByParam(_sdf, this->joint_RHP_,      "RHP") &&
          this->FindJointByParam(_sdf, this->joint_RHR_,      "RHR") &&
          this->FindJointByParam(_sdf, this->joint_RHY_,      "RHY"))

        return true;
      else
        return false;
    }

    public: bool FindJointByParam(sdf::ElementPtr _sdf,
                                  physics::JointPtr &_joint,
                                  std::string _param)
    {
      if (!_sdf->HasElement(_param))
      {
        gzerr << "param [" << _param << "] not found\n";
        return false;
      }
      else
      {
        _joint = this->model->GetJoint(
          _sdf->GetElement(_param)->GetValueString());

        if (!_joint)
        {
          gzerr << "joint by name ["
                << _sdf->GetElement(_param)->GetValueString()
                << "] not found in model\n";
          return false;
        }
      }
      return true;
    }

    // Called by the world update start event
    public: void OnUpdate()
    {
      double maxTorque = 500;
      double iniAngle = 0;

      /* Get the current feed-forward (state) */
      int r = ach_get(&chan_hubo_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_LAST );
      if(ACH_OK != r) {
        this->joint_RSP_->SetAngle(0, H_ref.ref[RSP]);
        printf("posrsp = %f \n\r",H_ref.ref[RSP]);
        sleep(0.25);
      }

      this->joint_LSP_->SetMaxForce(0, maxTorque);
      this->joint_LSR_->SetMaxForce(0, maxTorque);
      this->joint_LSY_->SetMaxForce(0, maxTorque);
      this->joint_LEP_->SetMaxForce(0, maxTorque);
      this->joint_LWP_->SetMaxForce(0, maxTorque);
      this->joint_LWR_->SetMaxForce(0, maxTorque);
      this->joint_LWY_->SetMaxForce(0, maxTorque);

      this->joint_RSP_->SetMaxForce(0, maxTorque);
      this->joint_RSR_->SetMaxForce(0, maxTorque);
      this->joint_RSY_->SetMaxForce(0, maxTorque);
      this->joint_REP_->SetMaxForce(0, maxTorque);
      this->joint_RWP_->SetMaxForce(0, maxTorque);
      this->joint_RWR_->SetMaxForce(0, maxTorque);
      this->joint_RWY_->SetMaxForce(0, maxTorque);

      this->joint_LAP_->SetMaxForce(0, maxTorque);
      this->joint_LAR_->SetMaxForce(0, maxTorque);
      this->joint_LKP_->SetMaxForce(0, maxTorque);
      this->joint_LHP_->SetMaxForce(0, maxTorque);
      this->joint_LHR_->SetMaxForce(0, maxTorque);
      this->joint_LHY_->SetMaxForce(0, maxTorque);

      this->joint_RAP_->SetMaxForce(0, maxTorque);
      this->joint_RAR_->SetMaxForce(0, maxTorque);
      this->joint_RKP_->SetMaxForce(0, maxTorque);
      this->joint_RHP_->SetMaxForce(0, maxTorque);
      this->joint_RHR_->SetMaxForce(0, maxTorque);
      this->joint_RHY_->SetMaxForce(0, maxTorque);

      this->joint_NK1_->SetMaxForce(0, maxTorque);
      this->joint_NK2_->SetMaxForce(0, maxTorque);
      this->joint_NK3_->SetMaxForce(0, maxTorque);

      this->joint_TSY_->SetMaxForce(0, maxTorque);

      this->joint_LF1_->SetMaxForce(0, maxTorque);
      this->joint_LF12_->SetMaxForce(0, maxTorque);
      this->joint_LF13_->SetMaxForce(0, maxTorque);
      this->joint_LF21_->SetMaxForce(0, maxTorque);
      this->joint_LF22_->SetMaxForce(0, maxTorque);
      this->joint_LF23_->SetMaxForce(0, maxTorque);
      this->joint_LF31_->SetMaxForce(0, maxTorque);
      this->joint_LF32_->SetMaxForce(0, maxTorque);
      this->joint_LF33_->SetMaxForce(0, maxTorque);

      this->joint_RF1_->SetMaxForce(0, maxTorque);
      this->joint_RF12_->SetMaxForce(0, maxTorque);
      this->joint_RF13_->SetMaxForce(0, maxTorque);
      this->joint_RF2_->SetMaxForce(0, maxTorque);
      this->joint_RF21_->SetMaxForce(0, maxTorque);
      this->joint_RF22_->SetMaxForce(0, maxTorque);
      this->joint_RF23_->SetMaxForce(0, maxTorque);
      this->joint_RF31_->SetMaxForce(0, maxTorque);
      this->joint_RF32_->SetMaxForce(0, maxTorque);
      this->joint_RF33_->SetMaxForce(0, maxTorque);


      this->joint_LSP_->SetAngle(0, iniAngle);
      this->joint_LSR_->SetAngle(0, iniAngle);
      this->joint_LSY_->SetAngle(0, iniAngle);
      this->joint_LEP_->SetAngle(0, iniAngle);
      this->joint_LWP_->SetAngle(0, iniAngle);
      this->joint_LWR_->SetAngle(0, iniAngle);
      this->joint_LWY_->SetAngle(0, iniAngle);
      
      this->joint_RSR_->SetAngle(0, iniAngle);
      this->joint_RSY_->SetAngle(0, iniAngle);
      this->joint_REP_->SetAngle(0, iniAngle);
      this->joint_RWP_->SetAngle(0, iniAngle);
      this->joint_RWR_->SetAngle(0, iniAngle);
      this->joint_RWY_->SetAngle(0, iniAngle);
      
      this->joint_LAP_->SetAngle(0, iniAngle);
      this->joint_LAR_->SetAngle(0, iniAngle);
      this->joint_LKP_->SetAngle(0, iniAngle);
      this->joint_LHP_->SetAngle(0, iniAngle);
      this->joint_LHR_->SetAngle(0, iniAngle);
      this->joint_LHY_->SetAngle(0, iniAngle);
      
      this->joint_RAP_->SetAngle(0, iniAngle);
      this->joint_RAR_->SetAngle(0, iniAngle);
      this->joint_RKP_->SetAngle(0, iniAngle);
      this->joint_RHP_->SetAngle(0, iniAngle);
      this->joint_RHR_->SetAngle(0, iniAngle);
      this->joint_RHY_->SetAngle(0, iniAngle);

      this->joint_NK1_->SetAngle(0, iniAngle);
      this->joint_NK2_->SetAngle(0, iniAngle);
      this->joint_NK3_->SetAngle(0, iniAngle);
 
      this->joint_TSY_->SetAngle(0, iniAngle);

      this->joint_LF1_->SetAngle(0, iniAngle);
      this->joint_LF12_->SetAngle(0, iniAngle);
      this->joint_LF13_->SetAngle(0, iniAngle);

      this->joint_LF21_->SetAngle(0, iniAngle);
      this->joint_LF22_->SetAngle(0, iniAngle);
      this->joint_LF23_->SetAngle(0, iniAngle);

      this->joint_LF31_->SetAngle(0, iniAngle);
      this->joint_LF32_->SetAngle(0, iniAngle);
      this->joint_LF33_->SetAngle(0, iniAngle);

      this->joint_RF1_->SetAngle(0, iniAngle);

      this->joint_RF12_->SetAngle(0, iniAngle);
      this->joint_RF13_->SetAngle(0, iniAngle);
      this->joint_RF2_->SetAngle(0, iniAngle);
      this->joint_RF21_->SetAngle(0, iniAngle);
      this->joint_RF22_->SetAngle(0, iniAngle);
      this->joint_RF23_->SetAngle(0, iniAngle);
      this->joint_RF31_->SetAngle(0, iniAngle);
      this->joint_RF32_->SetAngle(0, iniAngle);
      this->joint_RF33_->SetAngle(0, iniAngle);

/*  ---- Ach Control ---
      // Get Ach chan data
    size_t fs;
    int r = ach_get( &chan_diff_drive_ref, &H_ref, sizeof(H_ref), &fs, NULL, ACH_O_LAST );

    if(ACH_OK != r | ACH_STALE_FRAMES != r | ACH_MISSED_FRAME != r) {
                if(debug) {
                  //      printf("Ref ini r = %s\n",ach_result_to_string(r));}
                   printf("ref int r = %i \n\r",r);
		 }
        }
        else{   assert( sizeof(H_ref) == fs ); }

      ttime = this->world->GetSimTime().Double();*/


    }

    // Pointer to the model
    private: physics::ModelPtr model;
    private: physics::WorldPtr world;

    // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: physics::JointPtr joint_LSP_;
    private: physics::JointPtr joint_LSR_;
    private: physics::JointPtr joint_LSY_;
    private: physics::JointPtr joint_LEP_;
    private: physics::JointPtr joint_LWP_;
    private: physics::JointPtr joint_LWR_;
    private: physics::JointPtr joint_LWY_;

    private: physics::JointPtr joint_RSP_;
    private: physics::JointPtr joint_RSR_;
    private: physics::JointPtr joint_RSY_;
    private: physics::JointPtr joint_REP_;
    private: physics::JointPtr joint_RWP_;
    private: physics::JointPtr joint_RWR_;
    private: physics::JointPtr joint_RWY_;

    private: physics::JointPtr joint_LAP_;
    private: physics::JointPtr joint_LAR_;
    private: physics::JointPtr joint_LKP_;
    private: physics::JointPtr joint_LHP_;
    private: physics::JointPtr joint_LHR_;
    private: physics::JointPtr joint_LHY_;
    
    private: physics::JointPtr joint_RAP_;
    private: physics::JointPtr joint_RAR_;
    private: physics::JointPtr joint_RKP_;
    private: physics::JointPtr joint_RHP_;
    private: physics::JointPtr joint_RHR_;
    private: physics::JointPtr joint_RHY_;

    private: physics::JointPtr joint_NK1_;
    private: physics::JointPtr joint_NK2_;
    private: physics::JointPtr joint_NK3_;

    private: physics::JointPtr joint_TSY_;

    private: physics::JointPtr joint_LF1_;
//    private: physics::JointPtr joint_LF11_;
    private: physics::JointPtr joint_LF12_;
    private: physics::JointPtr joint_LF13_;
//    private: physics::JointPtr joint_LF2_;
    private: physics::JointPtr joint_LF21_;
    private: physics::JointPtr joint_LF22_;
    private: physics::JointPtr joint_LF23_;
//    private: physics::JointPtr joint_LF3_;
    private: physics::JointPtr joint_LF31_;
    private: physics::JointPtr joint_LF32_;
    private: physics::JointPtr joint_LF33_;

    private: physics::JointPtr joint_RF1_;
//    private: physics::JointPtr joint_RF11_;
    private: physics::JointPtr joint_RF12_;
    private: physics::JointPtr joint_RF13_;
    private: physics::JointPtr joint_RF2_;
    private: physics::JointPtr joint_RF21_;
    private: physics::JointPtr joint_RF22_;
    private: physics::JointPtr joint_RF23_;
//    private: physics::JointPtr joint_RF3_;
    private: physics::JointPtr joint_RF31_;
    private: physics::JointPtr joint_RF32_;
    private: physics::JointPtr joint_RF33_;
//    private: physics::JointPtr joint_RF4_;
//    private: physics::JointPtr joint_RF41_;
//    private: physics::JointPtr joint_RF42_;
//    private: physics::JointPtr joint_RF43_;

  };

  // Register this plugin with the simulator
  GZ_REGISTER_MODEL_PLUGIN(DrcHuboPlugin)
}
