#include <Arduino.h>

class Insole
{
public:
    void INIT();
    void READ();
    int thr = 800; // the pressure sensor threshold
    double gait_percent_R = 0;
    double gait_percent_L = 0;
    double sample_rate;
    int pressure_status=0;
    double torque_profile[101]={-0.27413,-0.24369,-0.19062,-0.12299,-0.075635,-0.034765,0.028277,0.11475,0.20177,0.27485,0.33338,0.37781,0.40805,0.42557,0.43147,0.42643,0.41171,0.38947,0.36183,0.33046,0.29694,0.26258,0.22795,0.19318,0.15859,0.12477,0.091997,0.060882,0.031714,0.0038911,-0.023137,-0.049289,-0.074365,-0.098108,-0.12015,-0.14066,-0.16014,-0.17857,-0.19467,-0.20789,-0.21765,-0.22334,-0.22437,-0.22031,-0.21076,-0.19568,-0.17527,-0.15026,-0.12167,-0.090699,-0.058479,-0.025809,0.0064469,0.036514,0.062467,0.083834,0.10006,0.11,0.11248,0.10418,0.084071,0.056723,0.030563,0.024408,0.036581,0.053624,0.068522,0.074131,0.071069,0.062239,0.051011,0.040078,0.030575,0.022788,0.016379,0.01035,0.0041335,-0.0020572,-0.0081695,-0.014393,-0.020769,-0.027303,-0.033877,-0.040245,-0.046254,-0.05215,-0.058965,-0.067896,-0.080366,-0.097101,-0.11762,-0.14075,-0.16517,-0.19081,-0.21894,-0.25023,-0.28124,-0.30352,-0.30497,-0.2702,-0.2858};
    double normalized_torque_command_L=0,normalized_torque_command_R=0;
    int toe_R = 0, heel_R = 0, mid1_R = 0, mid2_R = 0, toe_L = 0, heel_L = 0, mid1_L = 0, mid2_L = 0;
    
private:
//    int toe_R = 0, heel_R = 0, mid1_R = 0, mid2_R = 0, toe_L = 0, heel_L = 0, mid1_L = 0, mid2_L = 0;
    double P_R_o = 0, P_L_o = 0, pre_P_R_o, pre_P_L_o; // original phase and previous phase status
    double P_R =0, P_L = 0, pre_P_L, pre_P_R; //debounced current and previous phase status 
    int itr_L=0, itr_R=0; //number of the cumulation points for the same gait phase
    int mini_interval; // the interval is used for debouncing the minimal time for status changing is 0.05s
    int step_arr_L[3],step_arr_R[3];
    int step_length;
    int L_index,R_index;
    int i_step_L,i_step_R;
    int step_L,step_R;
    int step_L_sum,step_R_sum;
    int gait_percent_floor=0;
    double ratio_left=0;
    double ratio_right=0;
    double Cur_command_L=0;
    double Cur_command_R=0;

};
