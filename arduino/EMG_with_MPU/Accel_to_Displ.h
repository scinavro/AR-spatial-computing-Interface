class AccelToDispl {
private:
  float temp_X[2][3] = {0,}, temp_Y[2][3] = {0,}, temp_Z[2][3] = {0,};
  float EMA_X[2][3] = {0,}, EMA_Y[2][3] = {0,}, EMA_Z[2][3] = {0,};
  float EMA_a[3] = {0.1, 0.1, 0.2};  // 클수록 highpass filter 경계 주파수 증가 (0 ~ 1)
  float delta_t;

public:
  float X[3], Y[3], Z[3]; // ?[0]: 가속도, ?[1]: 속도, ?[2]: 변위
  

  AccelToDispl(float delta_t_) : delta_t(delta_t_) {}
  void getAccelData(float accelX_, float accelY_, float accelZ_);
  void filter(int mode);
  void integral(int mode);
};

void AccelToDispl ::getAccelData(float accelX_, float accelY_, float accelZ_){
  X[0] = accelX_;
  Y[0] = accelY_;
  Z[0] = accelZ_;
}

// high-pass filter
// mode 0: 가속도 필터링,  mode 1: 속도 필터링, mode 2: 변위 필터링
void AccelToDispl ::filter(int mode) {
  temp_X[0][mode] += X[mode] * delta_t;
  temp_Y[0][mode] += Y[mode] * delta_t;
  temp_Z[0][mode] += Z[mode] * delta_t;

  EMA_X[0][mode] = (EMA_a[mode] * temp_X[0][mode]) + ((1 - EMA_a[mode]) * EMA_X[0][mode]);
  EMA_Y[0][mode] = (EMA_a[mode] * temp_Y[0][mode]) + ((1 - EMA_a[mode]) * EMA_Y[0][mode]);
  EMA_Z[0][mode] = (EMA_a[mode] * temp_Z[0][mode]) + ((1 - EMA_a[mode]) * EMA_Z[0][mode]);

  X[mode] = temp_X[0][mode] - EMA_X[0][mode];
  Y[mode] = temp_Y[0][mode] - EMA_Y[0][mode];
  Z[mode] = temp_Z[0][mode] - EMA_Z[0][mode];

  if (mode == 0){
    temp_X[1][mode] += X[mode] * delta_t;
    temp_Y[1][mode] += Y[mode] * delta_t;
    temp_Z[1][mode] += Z[mode] * delta_t;

    EMA_X[1][mode] = (EMA_a[mode] * temp_X[1][mode]) + ((1 - EMA_a[mode]) * EMA_X[1][mode]);
    EMA_Y[1][mode] = (EMA_a[mode] * temp_Y[1][mode]) + ((1 - EMA_a[mode]) * EMA_Y[1][mode]);
    EMA_Z[1][mode] = (EMA_a[mode] * temp_Z[1][mode]) + ((1 - EMA_a[mode]) * EMA_Z[1][mode]);

    X[mode] = temp_X[1][mode] - EMA_X[1][mode];
    Y[mode] = temp_Y[1][mode] - EMA_Y[1][mode];
    Z[mode] = temp_Z[1][mode] - EMA_Z[1][mode];
  }
}

// mode 1: 가속도 -> 속도,  mode 2: 속도 -> 변위
void AccelToDispl ::integral(int mode) {
  X[mode] += X[mode-1] * delta_t;
  Y[mode] += Y[mode-1] * delta_t;
  Z[mode] += Z[mode-1] * delta_t;
}