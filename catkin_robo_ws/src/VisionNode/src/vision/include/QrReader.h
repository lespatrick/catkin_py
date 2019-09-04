//
//  QrReader.hpp
//  Vision
//
//  Created by Leszek Barszcz on 12/05/2019.
//  Copyright © 2019 lpb. All rights reserved.
//

#ifndef QrReader_hpp
#define QrReader_hpp

#include <stdio.h>

#include "zbar.h"
#include "boost/unordered_map.hpp"
#include "boost/date_time/posix_time/posix_time.hpp"

class QrReader {
public:
    QrReader() {};
    void processImage(zbar::Image &zbar_image, std::function<void(std::string, int, int)> recognisedBarcode);

private:
    zbar::ImageScanner scanner_;
    boost::unordered_map<std::string, boost::posix_time::ptime> barcode_memory_;

    double throttle_;
};

#endif /* QrReader_hpp */
