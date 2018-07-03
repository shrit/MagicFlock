/**
 * @file controller.hh
 * @brief controller code, that allow user to control the quad
 *
 * @authors Author: Omar Shrit <shrit@lri.fr>
 * @date 2018-06-13
 */

# include <chrono>
# include <cmath>
# include <dronecore/action.h>
# include <dronecore/telemetry.h>
# include <dronecore/dronecore.h>
# include <dronecore/offboard.h>
# include <iostream>
# include <thread>
# include <QtCore/QObject>
# include <QtCore/QTimer>




class QGamepad;

class Controller
  : public  QObject, public std::enable_shared_from_this<Controller>  
{

  Q_OBJECT


public:

  explicit Controller( QObject* parent = 0
	     



	     );

  ~Controller();

  bool takeoff(std::shared_ptr<dronecore::Action> action);
  bool land(std::shared_ptr<dronecore::Action> action);
  bool goUp(std::shared_ptr<dronecore::Offboard> offboard);
  bool goDown(std::shared_ptr<dronecore::Offboard> offboard);
  bool goRight(std::shared_ptr<dronecore::Offboard> offboard);
  bool goLeft(std::shared_ptr<dronecore::Offboard> offboard);
  bool forward(std::shared_ptr<dronecore::Offboard> offboard);
  bool backward(std::shared_ptr<dronecore::Offboard> offboard);
  bool turnToLeft(std::shared_ptr<dronecore::Offboard> offboard);
  bool turnToRight(std::shared_ptr<dronecore::Offboard> offboard);
  
  bool discover_system(DroneCore dc);
  ConnectionResult connect(DroneCore dc, std::string connection_url);
  ActionResult arm(DroneCore dc);
  
  void get_position(std::shared_ptr<dronecore::Telemetr> telemetry);
  void quad_health(std::shared_ptr<dronecore::Telemetr> telemetry);
  
 

private:
  DroneCore dc_;
  
  std::shared_ptr<dronecore::Telemetry>(system) telemetry_;
  std::shared_ptr<dronecore::Action>(system) action_;
  std::shared_ptr<dronecore::Offboard>(system) offboard_;

  QGamepad* m_gamepad_;

};



  
