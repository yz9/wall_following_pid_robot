#ifndef __COMP417_PID_H__
#define __COMP417_PID_H__


class PID {
 public:
  PID(double Kp, double Td, double Ti, double dt) {
    // todo: write this
  };
  
  void update_control(double current_error) {
    // todo: write this
  };

  double get_control() {
    return control;
  };

 private:
  double Kp;
  double Td;
  double Ti;

  double current_error;
  double previous_error;

  double sum_error;
  
  double current_deriv_error;
  double previous_deriv_error;
  double control;
  double dt;
};

#endif
