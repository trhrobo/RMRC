class dynamixel {
private:
  int id;

public:
  dynamixel(int user_id);
  ~dynamixel();
  double angleCal(double goal_value) { return goal_value; }
};

