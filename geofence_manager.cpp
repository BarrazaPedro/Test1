#include "geofence_manager.h"

#define G 1000000000
#define M 1000000
#define K 1000

enum GEOSTATUS {
  OUTSIDE = 0,
  SAFE    = 1,
  UNSAFE  = 2,
  SET     = 3,
  FREE    = 5,
  IGNORE  = 8
};

uint8_t   geostatus;
double    nearest_distance;
uint16_t  nearest_geofence;

geofence_manager::geofence_manager() {
}

geofence_type* geofence_manager::create_container(void)
{
  geofence_container = new geofence_type[CONTAINER_SIZE]();
  if (geofence_container == nullptr) {
    Serial.println("Error: Container creation failed");
  }
  return geofence_container;
}

void geofence_manager::delete_container(void)
{
  if (geofence_container != nullptr) {
    delete[] geofence_container;
    geofence_container = nullptr;
  } else {
    Serial.println("Error: Trying to delete an empty container");
  }
}

void geofence_manager::fill_container(uint16_t offset, const uint8_t* data, uint16_t data_size)
{
  if (geofence_container == nullptr || data == nullptr) {
    Serial.println("Error: Container or array do not exist.");
    return;
  }
  if (offset + data_size / GEOFENCE_SIZE > CONTAINER_SIZE) {
    Serial.println("Error: Offset exceeds the size of the container");
    return;
  }
  for (uint16_t i = 0; i < data_size; i += GEOFENCE_SIZE)
  {
    geofence_container[i / GEOFENCE_SIZE + offset].addr = offset * GEOFENCE_SIZE + i;
    geofence_container[i / GEOFENCE_SIZE + offset].type = data[i + 3];
    if (data[i + 3] != CLEAR) {
      geofence_container[i / GEOFENCE_SIZE + offset].radius = data[i + 4] << 24 | data[i + 5] << 16 | data[i + 6] << 8 | data[i + 7];
      uint32_t lat = data[i + 8] << 24 | data[i + 9] << 16 | data[i + 10] << 8 | data[i + 11];
      geofence_container[i / GEOFENCE_SIZE + offset].latitude = convert_uint32_to_double(lat);
      uint32_t lng = data[i + 12] << 24 | data[i + 13] << 16 | data[i + 14] << 8 | data[i + 15];
      geofence_container[i / GEOFENCE_SIZE + offset].longitude = convert_uint32_to_double(lng);
    } else {
      geofence_container[i / GEOFENCE_SIZE + offset].radius = 0;
      geofence_container[i / GEOFENCE_SIZE + offset].latitude = 0;
      geofence_container[i / GEOFENCE_SIZE + offset].longitude = 0;
    }
  }
}

double geofence_manager::convert_uint32_to_double(uint32_t uint_value)
{
  int sign = 1;
  if (uint_value >= G) {
    sign = -1;
    uint_value -= G;
  }
  double double_value = sign * (double)uint_value / M;
  return double_value;
}

uint32_t geofence_manager::convert_double_to_uint32(double double_value)
{
  if (double_value < 0) {
    double_value = -1 * double_value + K;
  }
  uint32_t uint_value = double_value * M;
  return uint_value;
}

void geofence_manager::convert_uint32_to_uint8(uint32_t uint32_val, uint8_t* uint8_val, uint16_t offset)
{
  uint8_val[offset] = uint32_val >> 24;
  uint8_val[offset + 1] = uint32_val >> 16;
  uint8_val[offset + 2] = uint32_val >> 8;
  uint8_val[offset + 3] = uint32_val;
}

uint16_t geofence_manager::container_count(void)
{
  uint16_t count = 0;
  for (int i = 0; i < CONTAINER_SIZE; i++)
  {
    if (geofence_container[i].type != CLEAR) {
      count++;
    }
  }
  return count;
}

uint16_t geofence_manager::seek_geofence_empty_pos()
{
  for (int i = 0; i < CONTAINER_SIZE; i++)
  {
    if (geofence_container[i].type == CLEAR) {
      Serial.printf("GeofenceNum:%d|", i);
      Serial.printf("type:%d|", geofence_container[i].type);
      Serial.printf("rad:%d|", geofence_container[i].radius);
      Serial.printf("lat:%f|", geofence_container[i].latitude);
      Serial.printf("lng:%f\n", geofence_container[i].longitude);
      return i * GEOFENCE_SIZE;
    }
  }
  return -1;
}

uint16_t geofence_manager::seek_geofence_mem_pos(double lat, double lng)
{
  for (int i = 0; i < CONTAINER_SIZE; i++)
  {
    if (lat == geofence_container[i].latitude && lng == geofence_container[i].longitude) {
      return i * GEOFENCE_SIZE;
    }
  }
  return -1;
}

void geofence_manager::print_geofence(uint16_t position)
{
  if (position >= CONTAINER_SIZE) {
    Serial.println("Incorrect Position");
    return;
  }
  Serial.printf("GeofenceNum:%d|", position);
  Serial.printf("type:%d|", geofence_container[position].type);
  Serial.printf("rad:%d|", geofence_container[position].radius);
  Serial.printf("lat:%f|", geofence_container[position].latitude);
  Serial.printf("lng:%f\n", geofence_container[position].longitude);
}

void geofence_manager::return_geofence(uint16_t position, geofence_type* geofence2return)
{
  if (position >= CONTAINER_SIZE) {
    Serial.println("Incorrect Position");
    return;
  }
  geofence2return->addr = geofence_container[position].addr;
  geofence2return->type = geofence_container[position].type;
  geofence2return->radius = geofence_container[position].radius;
  geofence2return->latitude = geofence_container[position].latitude;
  geofence2return->longitude = geofence_container[position].longitude;
}

void geofence_manager::print_all_geofences(void)
{
  for (int i = 0; i < CONTAINER_SIZE; i++)
  {
    if (geofence_container[i].type != CLEAR) {
      Serial.printf("GeofenceNum:%d|", i);
      Serial.printf("type:%d|", geofence_container[i].type);
      Serial.printf("rad:%d|", geofence_container[i].radius);
      Serial.printf("lat:%f|", geofence_container[i].latitude);
      Serial.printf("lng:%f\n", geofence_container[i].longitude);
    }
  }
}

double geofence_manager::calculate_distance(double lat1, double lng1, double lat2, double lng2)
{
  double delta_lng = radians(lng1 - lng2);
  double sin_lng = sin(delta_lng);
  double cos_lng = cos(delta_lng);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double sin_lat1 = sin(lat1);
  double cos_lat1 = cos(lat1);
  double sin_lat2 = sin(lat2);
  double cos_lat2 = cos(lat2);
  double delta = (cos_lat1 * sin_lat2) - (sin_lat1 * cos_lat2 * cos_lng);
  delta = sq(delta);
  delta += sq(cos_lat2 * sin_lng);
  delta = sqrt(delta);
  double denominator = (sin_lat1 * sin_lat2) + (cos_lat1 * cos_lat2 * cos_lng);
  delta = atan2(delta, denominator);
  return delta * 6372795;
}

uint16_t geofence_manager::compare_distance(double lat, double lng)
{
  uint16_t shorter_geofence = -1;
  double shorter_distance = UINT32_MAX;
  nearest_distance = 20 * M;
  for (int i = 0; i < CONTAINER_SIZE; i++)
  {
    if (geofence_container[i].type != CLEAR) {
      double distance = calculate_distance(lat, lng, geofence_container[i].latitude, geofence_container[i].longitude);
      if (distance < nearest_distance) {
        nearest_distance = distance;
      }
      if (distance < geofence_container[i].radius) {
        if (distance < shorter_distance) {
          shorter_geofence = i;
          shorter_distance = distance;
        }
      }
    }
  }
  return shorter_geofence;
}

double geofence_manager::return_nearest_distance(void)
{
  return nearest_distance;
}

uint16_t geofence_manager::return_nearest_geofence(void)
{
  return nearest_geofence;
}

uint8_t geofence_manager::get_geostatus(bool enabler, double lat, double lng)
{
  geostatus = OUTSIDE;
  if (!enabler) {
    geostatus = IGNORE;
    return geostatus;
  }
  nearest_geofence = compare_distance(lat, lng);
  if (nearest_geofence != (uint16_t) - 1) {
    geostatus = geofence_container[nearest_geofence].type;
  }
  return geostatus;
}

uint8_t geofence_manager::return_geostatus(void)
{
  return geostatus;
}

void geofence_manager::check_heap_size(void)
{
  size_t lfb = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
  size_t tfs = heap_caps_get_free_size(MALLOC_CAP_8BIT);
  Serial.printf("Largest free block:%d|", lfb);
  Serial.printf("Total free size:%d\n", tfs);
}

uint8_t* geofence_manager::create_raw_array(void)
{
  geofence_raw_array = new uint8_t[4 * KB]();
  memset(geofence_raw_array, UINT8_MAX, 4 * KB);
  if (geofence_raw_array == nullptr) {
    Serial.println("Error: Array creation failed");
  }
  return geofence_raw_array;
}

void geofence_manager::delete_raw_array(uint8_t* &raw_array)
{
  if (raw_array != nullptr) {
    delete[] raw_array;
    raw_array = nullptr;
  } else {
    Serial.println("Error: Trying to delete an empty raw array");
  }
}

void geofence_manager::fill_raw_array(uint32_t uint32_val)
{
  uint16_t offset = seek_raw_array_empty_pos();
  convert_uint32_to_uint8(uint32_val, geofence_raw_array, offset);
}

void geofence_manager::clear_raw_array(void)
{
  for (int i = 0; i < 4 * KB; i++)
  {
    geofence_raw_array[i] = CLEAR;
  }
}

void geofence_manager::copy_raw_array(uint8_t* array_copy)
{
  for (int i = 0; i < 4 * KB; i++) {
    array_copy[i] = geofence_raw_array[i];
  }
}

void geofence_manager::read_raw_array(uint16_t addr2read)
{
  Serial.printf("DataPack:");
  for (int i = 0; i < 16; i++)
  {
    Serial.printf("%d,",  geofence_raw_array[addr2read + i]);
  }
  Serial.println();
}

uint16_t geofence_manager::seek_raw_array_empty_pos(void)
{
  for (int i = 0; i < 4 * KB; i += 4)
  {
    if (geofence_raw_array[i] == CLEAR) {
      return i;
    }
  }
  return -1;
}
