#include <stdexcept>
#include "../src/fresh_range_layer.cpp"

class TestLayer : public roport_range_layer::FreshRangeLayer
{
public:
  TestLayer()
  {
    default_value_ = 127;
    buffered_readings_ = 1;
    resizeMap(5, 5, 0.05, 0.0, 0.0);
    mark_threshold_ = 0.8;
  }
  void keep(double seconds) {observation_keep_time_ = seconds;}
  void observe(unsigned int x, unsigned int y, double stamp)
  {
    double wx, wy;
    mapToWorld(x, y, wx, wy);
    observed_at_[worldKey(wx, wy)] = stamp;
    setCost(x, y, 233);
  }
  bool expire(double stamp) {return expireObservations(stamp);}
};

int main()
{
  TestLayer layer;
  layer.setCost(2, 2, 254);
  layer.setCost(0, 0, 233);
  layer.clearArea(1, 1, 3, 3, false);
  if (layer.getCost(2, 2) != 127 || layer.getCost(0, 0) != 233) {
    throw std::runtime_error("clear must restore neutral prior only inside the requested area");
  }
  layer.setCost(2, 2, 233);
  layer.clearArea(1, 1, 3, 3, true);
  if (layer.getCost(2, 2) != 233 || layer.getCost(0, 0) != 127) {
    throw std::runtime_error("inverted clear must preserve the excluded area");
  }
  layer.clearArea(-1, -1, 6, 6, false);
  for (unsigned int y = 0; y < 5; ++y) {
    for (unsigned int x = 0; x < 5; ++x) {
      if (layer.getCost(x, y) != 127) {
        throw std::runtime_error("entire clear must leave no occupied or probability-one prior");
      }
    }
  }
  layer.keep(1.0);
  layer.observe(2, 2, 10.0);
  if (layer.expire(10.9) || layer.getCost(2, 2) != 233) {
    throw std::runtime_error("fresh obstacle must remain marked");
  }
  layer.observe(3, 3, 10.9);
  if (!layer.expire(11.1) || layer.getCost(2, 2) != 127 || layer.getCost(3, 3) != 233) {
    throw std::runtime_error("only unsupported historical obstacles must expire");
  }
  layer.keep(0.0);
  if (layer.expire(20.0) || layer.getCost(3, 3) != 233) {
    throw std::runtime_error("default persistence behavior must remain unchanged");
  }
  layer.keep(1.0);
  layer.observe(3, 3, 30.0);
  layer.updateOrigin(0.05, 0.0);
  layer.expire(30.5);
  if (layer.getCost(2, 3) != 233) {
    throw std::runtime_error("rolling origin must preserve a fresh world observation");
  }
  layer.expire(31.1);
  if (layer.getCost(2, 3) != 127) {
    throw std::runtime_error("rolling origin must not preserve an expired observation");
  }
}
