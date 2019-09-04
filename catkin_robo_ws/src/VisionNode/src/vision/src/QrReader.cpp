//
//  QrReader.cpp
//  Vision
//
//  Created by Leszek Barszcz on 12/05/2019.
//  Copyright © 2019 lpb. All rights reserved.
//

#include "QrReader.h"

void QrReader::processImage(zbar::Image &zbar_image, std::function<void(std::string, int, int)> recognisedBarcode) {
    using namespace boost::posix_time;

    scanner_.scan(zbar_image);

    // iterate over all barcode readings from image
    for (zbar::Image::SymbolIterator symbol = zbar_image.symbol_begin();
         symbol != zbar_image.symbol_end(); ++symbol) {
        std::string barcode = symbol->get_data();
        // verify if repeated barcode throttling is enabled
        if (throttle_ > 0.0) {
            // check if barcode has been recorded as seen, and skip detection
            if (barcode_memory_.count(barcode) > 0) {
                // check if time reached to forget barcode
                if (second_clock::local_time() > barcode_memory_.at(barcode)) {
                    barcode_memory_.erase(barcode);
                } else {
                    // if timeout not reached, skip this reading
                    continue;
                }
            }
            // record barcode as seen, with a timeout to 'forget'
            barcode_memory_.insert(std::make_pair(barcode, second_clock::local_time() + milliseconds(int(throttle_ * 1000))));
        }

        zbar::Symbol::Point minXPoint = *(symbol->point_begin());
        zbar::Symbol::Point minYPoint = *(symbol->point_begin());
        zbar::Symbol::Point maxXPoint = *(symbol->point_begin());
        zbar::Symbol::Point maxYPoint = *(symbol->point_begin());

        zbar::Symbol::PointIterator pointIter = symbol->point_begin();
        for (int i = 0; i < 4; i++) {
            zbar::Symbol::Point point = *pointIter;
            if (minXPoint.x > point.x) {
                minXPoint = point;
            } else if (minYPoint.y > point.y) {
                minYPoint = point;
            } else if (maxXPoint.x < point.x) {
                maxXPoint = point;
            } else if (maxYPoint.y < point.y) {
                maxYPoint = point;
            }
            ++pointIter;
        }

        int symbolWidth = maxXPoint.x - minXPoint.x;
        int symbolHeight = maxYPoint.y - minYPoint.y;

        if (recognisedBarcode) {
            recognisedBarcode(barcode, symbolWidth, symbolHeight);
        }
    }

    zbar_image.set_data(NULL, 0);
}