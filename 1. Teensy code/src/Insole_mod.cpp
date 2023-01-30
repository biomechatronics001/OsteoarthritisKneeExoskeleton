#include "Insole_mod.h"

void Insole::INIT()
{
  pinMode(A0, INPUT_PULLUP);
  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  pinMode(A3, INPUT_PULLUP);
  pinMode(A4, INPUT_PULLUP);
  pinMode(A5, INPUT_PULLUP);
  pinMode(A6, INPUT_PULLUP);
  pinMode(A7, INPUT_PULLUP);
  sample_rate = 200;
  step_length=3;
  mini_interval = sample_rate*0.05;
  for(int i=0;i<step_length;i++)
  {
    step_arr_L[i]={sample_rate};
    step_arr_R[i]={sample_rate};
  }
  L_index=0;
  R_index=0;
  i_step_L=0;
  i_step_R=0;
  step_L=0;
  step_R=0;
  step_L_sum=sample_rate*step_length;
  step_R_sum=sample_rate*step_length;
}

void Insole::READ()
{
  toe_L = analogRead(A7);
  heel_L = analogRead(A5);
  mid1_L = analogRead(A6);
  mid2_L = analogRead(A4);
  toe_R = analogRead(A2);
  heel_R = analogRead(A1);
  mid1_R = analogRead(A3);
  mid2_R = analogRead(A0); 
  if (toe_L < thr)
  {
    toe_L = 0;
  }
  else
  {
    toe_L = 1;
  }
  if (heel_L < thr)
  {
    heel_L = 0;
  }
  else
  {
    heel_L = 1;
  }
  if (mid1_L < thr)
  {
    mid1_L = 0;
  }
  else
  {
    mid1_L = 1;
  }
  if (mid2_L < thr)
  {
    mid2_L = 0;
  }
  else
  {
    mid2_L = 1;
  }
  if (toe_R < thr)
  {
    toe_R = 0;
  }
  else
  {
    toe_R = 1;
  }
  if (heel_R < thr)
  {
    heel_R = 0;
  }
  else
  {
    heel_R = 1;
  }
  if (mid1_R < thr)
  {
    mid1_R = 0;
  }
  else
  {
    mid1_R = 1;
  }
  if (mid2_R < thr)
  {
    mid2_R = 0;
  }
  else
  {
    mid2_R = 1;
  }
  pressure_status = (toe_L) | (heel_L << 1) | (mid1_L << 2) | (mid2_L << 3) | (toe_R << 4) | (heel_R << 5) | (mid1_R << 6) | (mid2_R << 7);
  P_L_o=heel_L;
  P_R_o=heel_R;
  // debouncing left
  if(P_L_o!=pre_P_L_o)
  {
    if(itr_L>mini_interval)
    {
      P_L=P_L_o;
      itr_L=0;
    }
    else
    {
      P_L=pre_P_L;
      itr_L++;
    }
  }
  else
  {
    P_L=pre_P_L;
    itr_L++;
  }  
   // debouncing right
  if(P_R_o!=pre_P_R_o)
  {
    if(itr_R>mini_interval)
    {
      P_R=P_R_o;
      itr_R=0;
    }
    else
    {
      P_R=pre_P_R;
      itr_R++;
    }
  }
  else
  {
    P_R=pre_P_R;
    itr_R++;
  }
//Left gait percentage calculation
  if((P_L-pre_P_L)==-1)
  {
     if(i_step_L<sample_rate*0.2)
    {
      step_L_sum=step_L_sum-step_arr_L[L_index]+sample_rate*0.2;
      i_step_L=sample_rate*0.2;
    }
    else
    {
      step_L_sum=step_L_sum-step_arr_L[L_index]+i_step_L;
      step_arr_L[L_index]=i_step_L;
    }
//    Serial.print(step_arr_L[0]);
//    Serial.print("  ");
//    Serial.print(step_arr_L[1]);
//    Serial.print("  ");
//    Serial.print(step_arr_L[2]);
//    Serial.print("  ");        
//    Serial.println(step_L_sum);
    step_L=step_L_sum/step_length;
    i_step_L=0;
    gait_percent_L=0;
    L_index++;
    L_index=L_index%step_length;
  }
  else
  {
     if(i_step_L<sample_rate*3)
     {
      gait_percent_L=((i_step_L*100.0)/step_L);
      i_step_L=i_step_L+1;
     }
     else
     {
       gait_percent_L=0;
     }
  }
  if(gait_percent_L>100)
  {gait_percent_L=100;}

//Right gait percentage calculation
  if((P_R-pre_P_R)==-1)
  {
    step_R_sum=step_R_sum-step_arr_R[R_index]+i_step_R;
    step_arr_R[R_index]=i_step_R;

    if(i_step_R<sample_rate*0.2)
    {
      step_R_sum=step_R_sum-step_arr_R[R_index]+sample_rate*0.2;
      i_step_R=sample_rate*0.2;
    }
    else
    {
      step_R_sum=step_R_sum-step_arr_R[R_index]+i_step_R;
      step_arr_R[R_index]=i_step_R;
    } 
    step_R=step_R_sum/step_length;
    i_step_R=0;
    gait_percent_R=0;
    R_index++;
    R_index=R_index%step_length;
  }
  else
  {
     if(i_step_R<sample_rate*3)
     {
      gait_percent_R=((i_step_R*100.0)/step_R);
      i_step_R=i_step_R+1;
     }
     else
     {
       gait_percent_R=0;
     }
  }
  if(gait_percent_R>100)
  {gait_percent_R=100;}
  pre_P_L_o=P_L_o;
  pre_P_R_o=P_R_o;      
  pre_P_L=P_L;
  pre_P_R=P_R;
  //mapping gait phase to torque
  gait_percent_floor=floor(gait_percent_L);
  ratio_left=gait_percent_L-gait_percent_floor;
  ratio_right=gait_percent_floor+1-gait_percent_L;
  normalized_torque_command_L=torque_profile[gait_percent_floor]*ratio_right+ torque_profile[gait_percent_floor+1]*ratio_left;
  gait_percent_floor=floor(gait_percent_R);
  ratio_left=gait_percent_R-gait_percent_floor;
  ratio_right=gait_percent_floor+1-gait_percent_R;
  normalized_torque_command_R=torque_profile[gait_percent_floor]*ratio_right+ torque_profile[gait_percent_floor+1]*ratio_left;
 

}
