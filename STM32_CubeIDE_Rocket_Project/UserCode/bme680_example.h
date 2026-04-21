/*
 * bme680_example.h
 *
 * Example usage of the BME680 driver.
 * Triggers a measurement every BME680_SAMPLE_INTERVAL_MS milliseconds and
 * prints temperature, pressure, humidity, and gas resistance to UART3
 * via myprintf().
 *
 * Integration:
 *   Call bme680_exampleInit() once during setup (USER CODE BEGIN 2).
 *   Call bme680_exampleUpdate() on every main loop iteration (USER CODE BEGIN 3).
 */

#ifndef BME680_EXAMPLE_H_
#define BME680_EXAMPLE_H_

void bme680_exampleInit(void);
void bme680_exampleUpdate(void);

#endif /* BME680_EXAMPLE_H_ */
