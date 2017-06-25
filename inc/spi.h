/*
 * spi.h
 *
 *  Created on: 4. 4. 2017
 *      Author: boris
 */

#ifndef SPI_H_
#define SPI_H_

#include <stdint.h>


/// write to sd card
///
/// @return	status from sd card.
uint32_t sendSD(const uint32_t toSend);



#endif /* SPI_H_ */
