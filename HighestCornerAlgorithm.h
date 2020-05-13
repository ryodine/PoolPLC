#ifndef HIGHEST_CORNER_ALGO_H
#define HIGHEST_CORNER_ALGO_H

class HighestCornerAlgo {
  public:
    HighestCornerAlgo(double hystLow, double hystHigh) 
      : corners({false, false, false, false}), lowerbound(hystLow), upperbound(hystHigh) {};
    void update(double roll, double pitch);

    bool getCorner(unsigned int corner, bool lowestCornerMode=false) { 
      return corners[(((lowestCornerMode)? 2 : 0) + corner) % 4]; 
    };
    
  private:
    bool corners[4];
    double lowerbound;
    double upperbound;
    void resetAll() {
      corners[0] = false;
      corners[1] = false;
      corners[2] = false;
      corners[3] = false;
    }
};

#endif
