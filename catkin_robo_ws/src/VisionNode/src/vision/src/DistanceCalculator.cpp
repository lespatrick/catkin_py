//
//  DistanceCalculator.cpp
//  Vision
//
//  Created by Leszek Barszcz on 13/05/2019.
//  Copyright © 2019 lpb. All rights reserved.
//

#include "DistanceCalculator.h"

double DistanceCalculator::calculateDistance(double sampledHeight) {
    return sampledHeight / modelHeight_ * modelDistance_;
}
