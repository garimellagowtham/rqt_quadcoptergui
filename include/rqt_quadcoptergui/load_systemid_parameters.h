#ifndef LOAD_SYSTEMID_PARAMETERS_H
#define LOAD_SYSTEMID_PARAMETERS_H

#include <Eigen/Dense>
#include <gcop/params.h>
#include <gcop/qrotorsystemid.h>
#ifndef ONEDEG
#define ONEDEG M_PI/180
#endif

namespace gcop {

using namespace std;

void loadParameters(std::string &file, QRotorSystemID &system_id)
{
    //DEBUG:
    cout<<"Param file: "<<file<<endl;
    Vector7d gain_stdev;
    Vector6d offsets_stdev;
    Params params(file.c_str());
    params.GetVectorXd("qrotor_gains_prior",system_id.qrotor_gains);
    params.GetVector6d("offsets_prior",system_id.offsets_prior);
    params.GetVectorXd("stdev_gains",gain_stdev);
    params.GetVectorXd("stdev_offsets",offsets_stdev);
    params.GetDouble("stdev_pos",system_id.stdev_position);
    params.GetDouble("stdev_rpy",system_id.stdev_rpy);
    params.GetVectorXd("stdev_initial_state",system_id.stdev_initial_state_prior);
    system_id.qrotor_gains_residualgain = gain_stdev.cwiseInverse().asDiagonal();
    system_id.offsets_prior_residualgain = offsets_stdev.cwiseInverse().asDiagonal();
    //Convert degrees to radians:
    system_id.stdev_rpy *= ONEDEG;
    system_id.stdev_initial_state_prior.tail<9>() *=  ONEDEG;
    cout<<"System Id Settings: "<<endl;
    cout<<"Gains: "<<system_id.qrotor_gains.transpose()<<endl;
    cout<<"Mean Offsets: "<<system_id.offsets_prior.transpose()<<endl;
    cout<<"Stdev Gains: "<<gain_stdev.transpose()<<endl;
    cout<<"Stdev Offsets: "<<offsets_stdev.transpose()<<endl;
    cout<<"Stdev Pos: "<<system_id.stdev_position<<endl;
    cout<<"Stdev RPY: "<<system_id.stdev_rpy<<endl;
    cout<<"Stdev Init State: "<<system_id.stdev_initial_state_prior.transpose()<<endl;
}

}

#endif // LOAD_SYSTEMID_PARAMETERS_H
