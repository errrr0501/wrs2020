#include <iostream>
#include "strategy_dual_arm/wipe_strategy.h"
//#include "strategy_dual_arm/main_strategy.hpp"



namespace strategy_dual_arm{

// // class MainStrategy : public WipeStrategy{
// class MainStrategy{

// public:
//     MainStrategy(int argc, char** argv);
//     ~MainStrategy();
// private:
//     WipeStrategy wipe_strategy;
 
// };
//MainStrategy::MainStrategy (argc,argv){}
// MainStrategy::MainStrategy(int argc, char** argv)
//   //: QMainWindow(parent)
//   : wipe_strategy(argc,argv)
//   //, strategy_msg(argc,argv)
// {
//     //WipeStrategy wipe_strategy;
//     // speed = 10;
//     // pos_x = 0.000;
//     // pos_y = 0.000;
//     // pos_z = 0.000;
//     // ori_roll   = 0.000;
//     // ori_pitch  = 0.000;
//     // ori_yaw    = 0.000;
//     // ori_phi    = 0.000;

//  // strategy_msg.init();


//}

//MainStrategy::~MainStrategy() {}

int main(int argc, char** argv){
    
    //MainStrategy MS;
    // strategy_dual_arm::WipeStrategy (argc,argv);
    // strategy_dual_arm::StrategyeMsg (argc,argv);
    // MainStrategy::~MainStrategy (argc,argv);
    //MainStrategy (argc,argv);
    //WipeStrategy::~WipeStrategy();

    
    WipeStrategy wipe_strategy();
    //WipeStrategy (argc,argv);
    //wipe_strategy.WipeStrategy (argc,argv);
    //StrategyeMsg (argc,argv);

    //StrategyeMsg.init();
    //WipeStrategy wipe;
    //WipeStrategy{};
    // speed = 50;
    // pos_x = -0.2000;
    // pos_y = 0.0363;
    // pos_z = -0.4750;
    // ori_roll   = 44.024;
    // ori_pitch  = -0.005;
    // ori_yaw    = -44.998;
    // ori_phi    = 0.000;
    // strategy_dual_arm::WipeStrategy::on_des_p2p();
    //WipeStrategy::on_des_p2p();

    wipe_strategy.on_des_p2p();

    return 0;
}
}
// #include <iostream>
// #include "strategy_dual_arm/test.h"
// using namespace std;

// int main()
// {
//  Circle c(3);
//  cout<<"Area="<<c.Area()<<endl;
//  return 1;
// }