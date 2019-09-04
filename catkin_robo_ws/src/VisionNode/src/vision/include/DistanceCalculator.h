//
//  DistanceCalculator.hpp
//  Vision
//
//  Created by Leszek Barszcz on 13/05/2019.
//  Copyright © 2019 lpb. All rights reserved.
//

#ifndef DistanceCalculator_hpp
#define DistanceCalculator_hpp

#include <stdio.h>

class DistanceCalculator {
public:
    DistanceCalculator(double modelHeight, double modelDistance) {
        modelHeight_ = modelHeight;
        modelDistance_ = modelDistance;
    }

    double calculateDistance(double sampledHeight);

private:
    double modelHeight_;
    double modelDistance_;
};

#endif /* DistanceCalculator_hpp */
