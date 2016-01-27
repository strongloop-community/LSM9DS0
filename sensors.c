/* Lot of the ideas and algos are originally from
 * https://github.com/sparkfun/LSM9DS0_Breakout/
 */

#include <getopt.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <inttypes.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <netinet/in.h>
#include <net/if.h>
#include <arpa/inet.h>
#include <bson.h>
#include <mongoc.h>
#include <MQTTClient.h>


#include "edison-9dof-i2c.h"

#define BYTE2BIN(byte) \
    (byte & 0x80 ? 1 : 0), \
    (byte & 0x40 ? 1 : 0), \
    (byte & 0x20 ? 1 : 0), \
    (byte & 0x10 ? 1 : 0), \
    (byte & 0x08 ? 1 : 0), \
    (byte & 0x04 ? 1 : 0), \
    (byte & 0x02 ? 1 : 0), \
    (byte & 0x01 ? 1 : 0)

uint8_t print_byte;
#define PRINT_REGISTER(file, address, reg) \
  read_byte(file, address, reg, &print_byte); \
  printf ("%-18s\t%02x / %d%d%d%d%d%d%d%d\n", \
          #reg":", print_byte, BYTE2BIN(print_byte))

#define ACC_GYRO_BIAS_FILENAME "acc-gyro.bias"
#define MAG_BIAS_FILENAME "mag.bias"

#define GYRO_ERROR M_PI * (40.0f / 180.0f) //rads/s
#define GYRO_DRIFT M_PI * (0.0f / 180.0f)  // rad/s/s
#define MADGWICK_BETA sqrt(3.0f / 4.0f) * GYRO_ERROR

// MQTT 

#define MQ_ADDR "tcp://localhost:1883"
#define MQ_ID "Edison"
#define MQ_PITCH_TOPIC "edison/pitch"
#define MQ_ROLL_TOPIC "edison/roll"
#define MQ_YAW_TOPIC "edison/yaw"
#define MQ_TEMP_TOPIC "edison/temperature"
#define MQ_QOS 1
#define MQ_TIMEOUT 10000L

      // MONGODB
#define MONGO_HOST "mongodb://localhost:27017"
      
      // COUCHBASE
#define COUCH_HOST "couchbase://localhost/"
      
#define DB_NAME "edison"
      
      
typedef struct {
    float x;
    float y;
    float z;
    float w;
} Quaternion;

static struct option long_options[] = {
  {"declination", required_argument, 0, 'd' },
  {"dump",        no_argument,       0, 'u' },
  {"help",        no_argument,       0, 'h' },
  {"mode",        required_argument, 0, 'm' },
  {"output",      required_argument, 0, 'o' },
  {"pitch",       no_argument,       0, 'p' },
  {"roll",        no_argument,       0, 'r' },
  {"yaw",         no_argument,       0, 'y' },
  {"gyro",        no_argument,       0, 'g' },
  {"magnetometer", no_argument,       0, 'm' },
  {"accelerometer", no_argument,      0, 'a' },
  {"sleepTime",    required_argument, 0, 't'},
  {"dbhost",       required_argument, 0, 'i'},
};

typedef enum {
  OPTION_MODE_SENSOR,
  OPTION_MODE_ANGLES,
  OPTION_MODE_PITCH,
  OPTION_MODE_ROLL,
  OPTION_MODE_YAW,
  OPTION_MODE_GYRO,
  OPTION_MODE_MAG,
  OPTION_MODE_ACC,
  OPTION_MODE_TXT,
  OPTION_MODE_JSON,
  OPTION_MODE_COUCH,
  OPTION_MODE_MONGO,
  OPTION_MODE_MQTT,
  OPTION_MODE_NORMAL,
} OptionMode;

OptionMode print_mode = OPTION_MODE_NORMAL;

void dump_config_registers (int file)
{
  printf (" * Non-output registers for %02x:\n", G_ADDRESS);
  PRINT_REGISTER (file, G_ADDRESS, WHO_AM_I_XM);
  PRINT_REGISTER (file, G_ADDRESS, CTRL_REG1_XM);
  PRINT_REGISTER (file, G_ADDRESS, CTRL_REG2_XM);
  PRINT_REGISTER (file, G_ADDRESS, CTRL_REG3_XM);
  PRINT_REGISTER (file, G_ADDRESS, CTRL_REG4_XM);
  PRINT_REGISTER (file, G_ADDRESS, CTRL_REG5_XM);
  PRINT_REGISTER (file, G_ADDRESS, REFERENCE_G);
  PRINT_REGISTER (file, G_ADDRESS, FIFO_CTRL_REG_G);
  PRINT_REGISTER (file, G_ADDRESS, INT1_CFG_G);
  PRINT_REGISTER (file, G_ADDRESS, INT1_TSH_XH_G);
  PRINT_REGISTER (file, G_ADDRESS, INT1_TSH_XL_G);
  PRINT_REGISTER (file, G_ADDRESS, INT1_TSH_YH_G);
  PRINT_REGISTER (file, G_ADDRESS, INT1_TSH_YL_G);
  PRINT_REGISTER (file, G_ADDRESS, INT1_TSH_ZH_G);
  PRINT_REGISTER (file, G_ADDRESS, INT1_TSH_ZL_G);
  PRINT_REGISTER (file, G_ADDRESS, INT1_DURATION_G);

  printf (" * Non-output registers for %02x:\n", XM_ADDRESS);
  PRINT_REGISTER (file, XM_ADDRESS, WHO_AM_I_XM);
  PRINT_REGISTER (file, XM_ADDRESS, INT_CTRL_REG_M);
  PRINT_REGISTER (file, XM_ADDRESS, INT_THS_L_M);
  PRINT_REGISTER (file, XM_ADDRESS, INT_THS_H_M);
  PRINT_REGISTER (file, XM_ADDRESS, OFFSET_X_L_M);
  PRINT_REGISTER (file, XM_ADDRESS, OFFSET_X_H_M);
  PRINT_REGISTER (file, XM_ADDRESS, OFFSET_Y_L_M);
  PRINT_REGISTER (file, XM_ADDRESS, OFFSET_Y_H_M);
  PRINT_REGISTER (file, XM_ADDRESS, OFFSET_Z_L_M);
  PRINT_REGISTER (file, XM_ADDRESS, OFFSET_Z_H_M);
  PRINT_REGISTER (file, XM_ADDRESS, REFERENCE_X);
  PRINT_REGISTER (file, XM_ADDRESS, REFERENCE_Y);
  PRINT_REGISTER (file, XM_ADDRESS, REFERENCE_Z);
  PRINT_REGISTER (file, XM_ADDRESS, CTRL_REG0_XM);
  PRINT_REGISTER (file, XM_ADDRESS, CTRL_REG1_XM);
  PRINT_REGISTER (file, XM_ADDRESS, CTRL_REG2_XM);
  PRINT_REGISTER (file, XM_ADDRESS, CTRL_REG3_XM);
  PRINT_REGISTER (file, XM_ADDRESS, CTRL_REG4_XM);
  PRINT_REGISTER (file, XM_ADDRESS, CTRL_REG5_XM);
  PRINT_REGISTER (file, XM_ADDRESS, CTRL_REG6_XM);
  PRINT_REGISTER (file, XM_ADDRESS, CTRL_REG7_XM);
  PRINT_REGISTER (file, XM_ADDRESS, FIFO_CTRL_REG);
  PRINT_REGISTER (file, XM_ADDRESS, INT_GEN_1_REG);
  PRINT_REGISTER (file, XM_ADDRESS, INT_GEN_1_THS);
  PRINT_REGISTER (file, XM_ADDRESS, INT_GEN_1_DURATION);
  PRINT_REGISTER (file, XM_ADDRESS, INT_GEN_2_REG);
  PRINT_REGISTER (file, XM_ADDRESS, INT_GEN_2_THS);
  PRINT_REGISTER (file, XM_ADDRESS, INT_GEN_2_DURATION);
  PRINT_REGISTER (file, XM_ADDRESS, CLICK_CFG);
  PRINT_REGISTER (file, XM_ADDRESS, CLICK_THS);
  PRINT_REGISTER (file, XM_ADDRESS, TIME_LIMIT);
  PRINT_REGISTER (file, XM_ADDRESS, TIME_LATENCY);
  PRINT_REGISTER (file, XM_ADDRESS, TIME_WINDOW);
  PRINT_REGISTER (file, XM_ADDRESS, ACT_THS);
  PRINT_REGISTER (file, XM_ADDRESS, ACT_DUR);
}

/* Axes for rotations (same as accelerometer and gyro measurements):
 *
 *  +----------------------+
 *  |                      |
 *  |   Intel    Edison    |  X-axis (pitch)
 *  |                      | --->
 *  |                      |
 *  | What  will you make? |
 *  +----------------------+
 *             |
 *             | Y-axis (roll)
 *             v
 *
 * Z-axis (yaw) downwards.
 *
 * The direction of rotation is clockwise when looking along the axis
 * from the origin (aka right hand rule). Roll and pitch are zero when
 * Edison is level, yaw is zero when y-axis points to north.
 * 
 */
void calculate_simple_angles (FTriplet mag, FTriplet acc, float declination, FTriplet *angles)
{
  float zz = acc.z * acc.z;

  /* See eq. 37 and 38 in
   * http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf */
  angles->x = -atan2(acc.y, sqrt(acc.x * acc.x + zz)) * (180.0 / M_PI);
  angles->y = atan2(acc.x, sqrt(acc.y * acc.y + zz)) * (180.0 / M_PI);

  angles->z = atan2 (mag.x, mag.y) * (180.0 / M_PI) - declination;
  if (angles->z > 180)
    angles->z -= 360;
  else if (angles->z < -180)
    angles->z += 360;
}

/* This function originally from 
 * https://github.com/sparkfun/LSM9DS0_Breakout/blob/master/Libraries/Arduino/SFE_LSM9DS0/examples/LSM9DS0_AHRS/LSM9DS0_AHRS.ino 
 * which in turn is an implementation of http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/ */
void madgwick_quaternion(FTriplet acc, FTriplet mag, FTriplet gyro, float deltat, Quaternion *quat)
{
    // short name local variable for readability
    float q1 = quat->x, q2 = quat->y, q3 = quat->z, q4 = quat->w;
    float ax = acc.x, ay = acc.y, az = acc.z;
    float mx = mag.x, my = mag.y, mz = mag.z;
    float gx = gyro.x * M_PI / 180.0,
          gy = gyro.y * M_PI / 180.0,
          gz = gyro.z * M_PI / 180.0;

    float norm;
    float hx, hy, _2bx, _2bz;
    float s1, s2, s3, s4;
    float qDot1, qDot2, qDot3, qDot4;

    // Auxiliary variables to avoid repeated arithmetic
    float _2q1mx;
    float _2q1my;
    float _2q1mz;
    float _2q2mx;
    float _4bx;
    float _4bz;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _2q4 = 2.0f * q4;
    float _2q1q3 = 2.0f * q1 * q3;
    float _2q3q4 = 2.0f * q3 * q4;
    float q1q1 = q1 * q1;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q1q4 = q1 * q4;
    float q2q2 = q2 * q2;
    float q2q3 = q2 * q3;
    float q2q4 = q2 * q4;
    float q3q3 = q3 * q3;
    float q3q4 = q3 * q4;
    float q4q4 = q4 * q4;

    // Normalise accelerometer measurement
    norm = sqrt(ax * ax + ay * ay + az * az);
    if (norm == 0.0f)
      return; // handle NaN

    norm = 1.0f/norm;
    ax *= norm;
    ay *= norm;
    az *= norm;

    // Normalise magnetometer measurement
    norm = sqrt(mx * mx + my * my + mz * mz);
    if (norm == 0.0f)
      return; // handle NaN

    norm = 1.0f/norm;
    mx *= norm;
    my *= norm;
    mz *= norm;

    // Reference direction of Earth's magnetic field
    _2q1mx = 2.0f * q1 * mx;
    _2q1my = 2.0f * q1 * my;
    _2q1mz = 2.0f * q1 * mz;
    _2q2mx = 2.0f * q2 * mx;
    hx = mx * q1q1 - _2q1my * q4 + _2q1mz * q3 + mx * q2q2 + _2q2 * my * q3 + _2q2 * mz * q4 - mx * q3q3 - mx * q4q4;
    hy = _2q1mx * q4 + my * q1q1 - _2q1mz * q2 + _2q2mx * q3 - my * q2q2 + my * q3q3 + _2q3 * mz * q4 - my * q4q4;
    _2bx = sqrt(hx * hx + hy * hy);
    _2bz = -_2q1mx * q3 + _2q1my * q2 + mz * q1q1 + _2q2mx * q4 - mz * q2q2 + _2q3 * my * q4 - mz * q3q3 + mz * q4q4;
    _4bx = 2.0f * _2bx;
    _4bz = 2.0f * _2bz;

    // Gradient decent algorithm corrective step
    s1 = -_2q3 * (2.0f * q2q4 - _2q1q3 - ax) + _2q2 * (2.0f * q1q2 + _2q3q4 - ay) - _2bz * q3 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q4 + _2bz * q2) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q3 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s2 = _2q4 * (2.0f * q2q4 - _2q1q3 - ax) + _2q1 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q2 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + _2bz * q4 * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q3 + _2bz * q1) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q4 - _4bz * q2) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s3 = -_2q1 * (2.0f * q2q4 - _2q1q3 - ax) + _2q4 * (2.0f * q1q2 + _2q3q4 - ay) - 4.0f * q3 * (1.0f - 2.0f * q2q2 - 2.0f * q3q3 - az) + (-_4bx * q3 - _2bz * q1) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (_2bx * q2 + _2bz * q4) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + (_2bx * q1 - _4bz * q3) * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    s4 = _2q2 * (2.0f * q2q4 - _2q1q3 - ax) + _2q3 * (2.0f * q1q2 + _2q3q4 - ay) + (-_4bx * q4 + _2bz * q2) * (_2bx * (0.5f - q3q3 - q4q4) + _2bz * (q2q4 - q1q3) - mx) + (-_2bx * q1 + _2bz * q3) * (_2bx * (q2q3 - q1q4) + _2bz * (q1q2 + q3q4) - my) + _2bx * q2 * (_2bx * (q1q3 + q2q4) + _2bz * (0.5f - q2q2 - q3q3) - mz);
    norm = sqrt(s1 * s1 + s2 * s2 + s3 * s3 + s4 * s4);    // normalise step magnitude
    norm = 1.0f/norm;
    s1 *= norm;
    s2 *= norm;
    s3 *= norm;
    s4 *= norm;

    // Compute rate of change of quaternion
    qDot1 = 0.5f * (-q2 * gx - q3 * gy - q4 * gz) - MADGWICK_BETA * s1;
    qDot2 = 0.5f * (q1 * gx + q3 * gz - q4 * gy) - MADGWICK_BETA * s2;
    qDot3 = 0.5f * (q1 * gy - q2 * gz + q4 * gx) - MADGWICK_BETA * s3;
    qDot4 = 0.5f * (q1 * gz + q2 * gy - q3 * gx) - MADGWICK_BETA * s4;

    // Integrate to yield quaternion
    q1 += qDot1 * deltat;
    q2 += qDot2 * deltat;
    q3 += qDot3 * deltat;
    q4 += qDot4 * deltat;
    norm = sqrt(q1 * q1 + q2 * q2 + q3 * q3 + q4 * q4);    // normalise quaternion
    norm = 1.0f/norm;
    quat->x = q1 * norm;
    quat->y = q2 * norm;
    quat->z = q3 * norm;
    quat->w = q4 * norm;
}

void calculate_tait_bryan_angles (Quaternion quat, float declination, FTriplet *angles)
{
    float yaw, pitch, roll;
    float q1 = quat.x, q2 = quat.y, q3 = quat.z, q4 = quat.w;

    yaw   = atan2(2.0f * (q2 * q3 + q1 * q4), q1 * q1 + q2 * q2 - q3 * q3 - q4 * q4);
    pitch = -asin(2.0f * (q2 * q4 - q1 * q3));
    roll  = atan2(2.0f * (q1 * q2 + q3 * q4), -(q1 * q1 - q2 * q2 - q3 * q3 + q4 * q4));

    angles->x = yaw * 180.0f / M_PI - declination; 
    angles->y = pitch * 180.0f / M_PI;
    angles->z = roll * 180.0f / M_PI;
}

int read_bias_files (Triplet *a_bias, Triplet *g_bias, Triplet *m_bias, FTriplet *m_scale)
{
  FILE *input;
  char g_str[7], a_str[7], m_str[7], m_scale_str[8];

  input = fopen(ACC_GYRO_BIAS_FILENAME, "r");
  if (input) {
    if (fscanf(input, "%s %hd %hd %hd",
               g_str, &g_bias->x, &g_bias->y, &g_bias->z) != 4 ||
        fscanf(input, "%s %hd %hd %hd",
               a_str, &a_bias->x, &a_bias->y, &a_bias->z) != 4 ||
        strcmp (g_str, "g_bias") != 0 ||
        strcmp (a_str, "a_bias") != 0) {
      if(print_mode == OPTION_MODE_NORMAL) printf ("Bias file "ACC_GYRO_BIAS_FILENAME" is malformed\n");
      fclose (input);
      return 0;
    } else {
      if(print_mode == OPTION_MODE_NORMAL) printf ("Loaded bias file G: %d %d %d, A: %d %d %d\n",
              g_bias->x, g_bias->y, g_bias->z,
              a_bias->x, a_bias->y, a_bias->z);
    }
    fclose (input);
  } else {
    if(print_mode == OPTION_MODE_NORMAL) printf ("Bias file "ACC_GYRO_BIAS_FILENAME" not found.\n");
  }

  input = fopen(MAG_BIAS_FILENAME, "r");
  if (input) {
    if (fscanf(input, "%s %hd %hd %hd",
               m_str, &m_bias->x, &m_bias->y, &m_bias->z) != 4 ||
        fscanf(input, "%s %f %f %f",
               m_scale_str, &m_scale->x, &m_scale->y, &m_scale->z) != 4 ||
        strcmp (m_str, "m_bias") != 0 ||
        strcmp (m_scale_str, "m_scale") != 0) {
      if(print_mode == OPTION_MODE_NORMAL) printf ("Bias file "MAG_BIAS_FILENAME" is malformed\n");
      fclose (input);
      return 0;
    } else {
      if(print_mode == OPTION_MODE_NORMAL) printf ("Loaded bias file M(bias): %d %d %d, M(scale): %4f %4f %4f\n",
              m_bias->x, m_bias->y, m_bias->z,
              m_scale->x, m_scale->y, m_scale->z);
    }
    fclose (input);
  } else {
    if(print_mode == OPTION_MODE_NORMAL) printf ("Bias file "MAG_BIAS_FILENAME" not found.\n");
  }

  return 1;
}

void mkfifo_at(char *dir_path, char *relative_path)
{
    int dir_fd;
    int error;
 
    dir_fd = open(dir_path, O_RDONLY);
    if (dir_fd < 0) {
        perror("open");
        exit(EXIT_FAILURE);
    }
 
    umask(0077);
    error = mkfifoat(dir_fd, relative_path, 0666);
    if (error != 0) {
        perror("mkfifoat");
        exit(EXIT_FAILURE);
    }
 
    close(dir_fd);
}

char * get_addr(){
    {
     int fd;
     struct ifreq ifr;
     char *addr;
     fd = socket(AF_INET, SOCK_DGRAM, 0);

     /* I want to get an IPv4 IP address */
     ifr.ifr_addr.sa_family = AF_INET;

     /* I want IP address attached to "eth0" */
     strncpy(ifr.ifr_name, "wlan0", IFNAMSIZ-1);

     ioctl(fd, SIOCGIFADDR, &ifr);

     close(fd);

     addr = malloc(100);
     /* display result */
     sprintf(addr, "%s", inet_ntoa(((struct sockaddr_in *)&ifr.ifr_addr)->sin_addr));

     return addr;
    }
}
int insert_mongo(char json[550], char *coll, mongoc_client_t *client, bson_oid_t oid){
    
    mongoc_collection_t *collection;
    bson_error_t error;
    bson_t *doc;
     
    collection = mongoc_client_get_collection (client, "edison", coll);     
    doc = bson_new_from_json((const uint8_t *)json, -1, &error);
    BSON_APPEND_OID (doc, "_id", &oid);
    if (!doc) {
		fprintf (stderr, "%s\n", error.message);
		return EXIT_FAILURE;
    }
    
    if (!mongoc_collection_insert (collection, MONGOC_INSERT_NONE, doc, NULL, &error)) {
		fprintf (stderr, "%s\n", error.message);
        return EXIT_FAILURE;
    }
    bson_destroy (doc);
    mongoc_collection_destroy (collection);
    return EXIT_SUCCESS;
}

int main (int argc, char **argv)
{
  int file;
  int16_t temp;
  uint8_t data[2] = {0};
  Triplet a_bias = {0}, g_bias = {0}, m_bias = {0};
  FTriplet m_scale;
  int opt, option_index, help = 0, option_dump = 0;
  OptionMode option_mode = OPTION_MODE_ANGLES;
  float declination = 0.0;
  int sleep_time = 1500;
  char server[100];
  char db_host[50];
  char date_string [50];
  char my_addr[50];

// Mongo DB
  mongoc_client_t *client;
  
  char json[550];
 bson_oid_t oid;
 
 sprintf(my_addr, "%s", get_addr());
 printf("Host Address: %s\n", my_addr);
//MQTT
  MQTTClient mqClient;
  MQTTClient_connectOptions conn_opts = MQTTClient_connectOptions_initializer;
  MQTTClient_message pubmessage = MQTTClient_message_initializer;
  MQTTClient_deliveryToken token;
  int rc;
  char tmp_msg[40];


  unlink("/tmp/gyro");
  unlink("/tmp/pitch");
  unlink("/tmp/roll");
  unlink("/tmp/yaw");
  unlink("/tmp/mag");
  unlink("/tmp/accel");
  mkfifo_at("/tmp", "gyro");
  mkfifo_at("/tmp", "pitch");
  mkfifo_at("/tmp", "roll");
  mkfifo_at("/tmp", "yaw");
  mkfifo_at("/tmp", "mag");
  mkfifo_at("/tmp", "accel");
  

  while ((opt = getopt_long(argc, argv, "d:hmp:u",
                            long_options, &option_index )) != -1) {
    switch (opt) {
      case 'd' :
        declination = atof (optarg);
        break;
      case 'm' :
        if (strcmp (optarg, "sensor") == 0)
          option_mode = OPTION_MODE_SENSOR;
        else if (strcmp (optarg, "angles") == 0)
          option_mode = OPTION_MODE_ANGLES;
        else
          help = 1;
        break;
      case 'o':
        if(strcmp(optarg, "mongo") == 0)
           print_mode = OPTION_MODE_MONGO;
        else if(strcmp(optarg, "couch") == 0)
           print_mode = OPTION_MODE_COUCH;
        else if(strcmp(optarg, "text") == 0)
           print_mode = OPTION_MODE_TXT;
        else if(strcmp(optarg, "json") == 0)
	   print_mode = OPTION_MODE_JSON;
        else if(strcmp(optarg, "mqtt") == 0)
           print_mode = OPTION_MODE_MQTT;
        else if (strcmp(optarg, "normal") == 0)
           print_mode = OPTION_MODE_NORMAL;
        else
           help = 1;
        break; 
      case 'i':
      
        sprintf(db_host, "%s", optarg);
        printf("DB Host: %s\n", db_host);
        break;  
      case 'u' :
        option_dump = 1;
        break;
      case 't' :
	    sleep_time = atoi(optarg) * 1000;
	    printf("Interval between readings will be: %d seconds.\n", atoi(optarg));
	    break;
      default:
        help = 1;
        break;
    }
  }

 

  if (help || argv[optind] != NULL) {
      printf ("%s [--mode <sensor|angles>] [--output <mongo|couch|text|mqtt|json|normal>] [--dbhost <hostname>][--sleepTime <seconds>] [--dump]\n", argv[0]);
      return 0;
  }

  if(print_mode == OPTION_MODE_MONGO){

  }

  if (!read_bias_files (&a_bias, &g_bias, &m_bias, &m_scale))
    return 1;

  file = init_device (I2C_DEV_NAME);
  if (file == 0)
    return 1;

  if (option_dump) {
    dump_config_registers(file);
    printf ("\n");
  }

  init_gyro(file, GYRO_SCALE_245DPS);
  init_mag(file, MAG_SCALE_2GS);
  init_acc(file, ACCEL_SCALE_2G);

  // int gpipe = open ("/tmp/gyro", O_WRONLY);
  // FILE *gyro_pipe = fdopen(gpipe, "+w");
  // printf("Gyro Pipe opened ... \n");

  // temperature is a 12-bit value: cut out 4 highest bits
  read_bytes (file, XM_ADDRESS, OUT_TEMP_L_XM, &data[0], 2);
  temp = (((data[1] & 0x0f) << 8) | data[0]);
  if(print_mode == OPTION_MODE_NORMAL)
	printf ("Temperature: %d\n", temp);

  if(print_mode == OPTION_MODE_MQTT){
      sprintf(server, "tcp://%s:1883", db_host);
	MQTTClient_create(&mqClient, server, MQ_ID,
        MQTTCLIENT_PERSISTENCE_NONE, NULL);
    	conn_opts.keepAliveInterval = 20;
    	conn_opts.cleansession = 1;
	if ((rc = MQTTClient_connect(mqClient, &conn_opts)) != MQTTCLIENT_SUCCESS) {
       		printf("Failed to connect, return code %d\n", rc);
       		exit(-1);
 	}
  } else if(print_mode == OPTION_MODE_MONGO){
      sprintf(server, "mongodb://%s:27017", db_host);
  } else if(print_mode == OPTION_MODE_COUCH){
      sprintf(server, "couchbase://%s/", db_host);
  } else {
      sprintf(server, "%s", db_host);
  }
  if (option_mode == OPTION_MODE_SENSOR) {
    if(print_mode == OPTION_MODE_NORMAL) printf ("  Gyroscope (deg/s)  | Magnetometer (mGs)  |   Accelerometer (mG)\n");
  } else {
    if(print_mode == OPTION_MODE_NORMAL) printf ("      Rotations (mag + acc):\n");
  }

  FTriplet angles2;
  while (1) {
    FTriplet gyro, mag, acc, angles1;

    usleep (sleep_time * 1000);
    time_t t = time(NULL);
    struct tm* tm_info;
    tm_info = localtime(&t);
    strftime(date_string, 26, "%m/%d/%Y %H:%M:%S", tm_info);
    read_gyro (file, g_bias, GYRO_SCALE_245DPS, &gyro);
    read_mag (file, m_bias, m_scale, MAG_SCALE_2GS, &mag);
    read_acc (file, a_bias, ACCEL_SCALE_2G, &acc);
    read_bytes (file, XM_ADDRESS, OUT_TEMP_L_XM, &data[0], 2);
    temp = (((data[1] & 0x0f) << 8) | data[0]);

    if (option_mode == OPTION_MODE_SENSOR) {
      printf ("gyro: %4.0f %4.0f %4.0f | ", gyro.x, gyro.y, gyro.z);
      // fprintf(gyro_pipe, "PIPE OUTPUT: \n");
     // fprintf(gyro_pipe, "gyro:  %4.0f %4.0f %4.0f\n", gyro.x, gyro.y, gyro.z);
     // fflush(gyro_pipe);
      // write(gyro_pipe, gyro_msg, strlen(gyro_msg) + 1);
      printf ("mag: %4.0f %4.0f %4.0f | ", mag.x*1000, mag.y*1000, mag.z*1000);
      printf ("acc: %4.0f %4.0f %5.0f\n", acc.x*1000, acc.y*1000, acc.z*1000);
    } else {
      calculate_simple_angles (mag, acc, declination, &angles1);
      if(angles1.x != angles2.x || angles1.y != angles2.y || angles1.z != angles2.z){
          
	  switch(print_mode) {
	     case OPTION_MODE_NORMAL :
                printf ("temp: %4d, pitch: %4.0f, roll: %4.0f, yaw: %4.0f\n",
                  temp, angles1.x, angles1.y, angles1.z);
                break;
	     case OPTION_MODE_TXT :
	        printf("%d; %d; %0.0f; %0.0f; %0.0f",  t, temp, angles1.x, angles1.y, angles1.x);
	        fflush(stdout);
		    break;	
	     case OPTION_MODE_JSON :
                printf("{\"time\": \"%s\",\"temp\": \"%d\",pitch\": \"%0.2f\",\"roll\": \"%0.2f\",\"yaw\": \"%0.2f\"}", date_string, temp, angles1.x, angles1.y, angles1.z);
		        fflush(stdout);
		break; 
	     case OPTION_MODE_MONGO : 
    		mongoc_init ();
    		client = mongoc_client_new (server);  
     bson_oid_init (&oid, NULL);
            

            // all sensor data          
		    sprintf(json, "{\"time\": \"%s\",\"temp\": \"%d\",\"pitch\": \"%0.2f\",\"roll\": \"%0.2f\",\"yaw\": \"%0.2f\", \"magX\": \"%0.2f\", \"magY\": \"%0.2f\", \"magZ\": \"%0.2f\", \"accelX\" : \"%0.2f\", \"accelY\" : \"%0.2f\", \"accelZ\" : \"%0.2f\"}", date_string, temp, angles1.x, angles1.y, angles1.z, mag.x*1000, mag.y*1000, mag.z*1000, acc.x*1000, acc.y*1000, acc.z*1000 );
            insert_mongo(json, "Sensor", client, oid);
            
            // Temperature
		    sprintf(json, "{\"time\": \"%s\",\"temp\": \"%d\"}", date_string, temp );
            insert_mongo(json, "Temperature", client, oid);
            
            // all Gyro
		    sprintf(json, "{\"time\": \"%s\",\"pitch\": \"%0.2f\",\"roll\": \"%0.2f\",\"yaw\": \"%0.2f\"}", date_string,  angles1.x, angles1.y, angles1.z );
            insert_mongo(json, "Gyroscope", client, oid);
            
            // pitch
		   sprintf(json, "{\"time\": \"%s\",\"pitch\": \"%0.2f\"}", date_string, angles1.x );
           insert_mongo(json, "Pitch", client, oid);
            
            // roll
		    sprintf(json, "{\"time\": \"%s\",\"roll\": \"%0.2f\"}", date_string, angles1.y );
            insert_mongo(json, "Roll", client, oid);
            
            //yaw
		    sprintf(json, "{\"time\": \"%s\",\"yaw\": \"%0.2f\"}", date_string, angles1.z);
            insert_mongo(json, "Yaw", client, oid);
            
            //All mag
		    sprintf(json, "{\"time\": \"%s\", \"magX\": \"%0.2f\", \"magY\": \"%0.2f\", \"magZ\": \"%0.2f\"}", date_string, mag.x*1000, mag.y*1000, mag.z*1000);
            insert_mongo(json, "Magnetometer", client, oid);
            
            // magX
		    sprintf(json, "{\"time\": \"%s\", \"magX\": \"%0.2f\"}", date_string, mag.x*1000);
            insert_mongo(json, "Magnetometer-X", client, oid);
            
            // magY
		    sprintf(json, "{\"time\": \"%s\",\"magY\": \"%0.2f\"}", date_string,  mag.y*1000);
            insert_mongo(json, "Magnetometer-Y", client, oid);
            
            // MagZ
		    sprintf(json, "{\"time\": \"%s\", \"magZ\": \"%0.2f\"}", date_string,  mag.z*1000 );
            insert_mongo(json, "Magnetometer-Z", client, oid);
            
            // All Accelerometer
		    sprintf(json, "{\"time\": \"%s\", \"accelX\" : \"%0.2f\", \"accelY\" : \"%0.2f\", \"accelZ\" : \"%0.2f\"}", date_string, acc.x*1000, acc.y*1000, acc.z*1000 );
            insert_mongo(json, "Accelerometer", client, oid);
            
            // accelX
		   sprintf(json, "{\"time\": \"%s\", \"accelX\" : \"%0.2f\"}", date_string, acc.x*1000 );
           insert_mongo(json, "Accelerometer-X", client, oid);
            
            // accelY
		    sprintf(json, "{\"time\": \"%s\", \"accelY\" : \"%0.2f\"}", date_string, acc.y*1000 );
            insert_mongo(json, "Accelerometer-Y", client, oid);
            
            // accelZ
		    sprintf(json, "{\"time\": \"%s\", \"accelZ\" : \"%0.2f\"}", date_string, acc.z*1000 );
            insert_mongo(json, "Accelerometer-Z", client, oid);
            
           
    		mongoc_client_destroy (client);
    		mongoc_cleanup ();
		break;
	     case OPTION_MODE_MQTT :
  		sprintf(tmp_msg, "Pitch: %0.4f", angles1.x);
    		pubmessage.payload = tmp_msg;
    		pubmessage.payloadlen = strlen(tmp_msg);
    		pubmessage.qos = MQ_QOS;
    		pubmessage.retained = 0;
    		MQTTClient_publishMessage(mqClient, MQ_PITCH_TOPIC, &pubmessage, &token);
    		printf("Waiting for up to %d seconds for publication of %s\n"
            		"on topic %s for client with ClientID: %s\n",
            		(int)(MQ_TIMEOUT/1000), tmp_msg, MQ_PITCH_TOPIC, MQ_ID);
    		rc = MQTTClient_waitForCompletion(mqClient, token, MQ_TIMEOUT);
    		printf("Message with delivery token %d delivered\n", token);

  		sprintf(tmp_msg, "Roll: %0.4f", angles1.y);
    		pubmessage.payload = tmp_msg;
    		pubmessage.payloadlen = strlen(tmp_msg);
    		pubmessage.qos = MQ_QOS;
    		pubmessage.retained = 0;
    		MQTTClient_publishMessage(mqClient, MQ_ROLL_TOPIC, &pubmessage, &token);
    		printf("Waiting for up to %d seconds for publication of %s\n"
            		"on topic %s for client with ClientID: %s\n",
            		(int)(MQ_TIMEOUT/1000), tmp_msg, MQ_ROLL_TOPIC, MQ_ID);
    		rc = MQTTClient_waitForCompletion(mqClient, token, MQ_TIMEOUT);
    		printf("Message with delivery token %d delivered\n", token);

  		sprintf(tmp_msg, "Yaw: %0.4f", angles1.z);
    		pubmessage.payload = tmp_msg;
    		pubmessage.payloadlen = strlen(tmp_msg);
    		pubmessage.qos = MQ_QOS;
    		pubmessage.retained = 0;
    		MQTTClient_publishMessage(mqClient, MQ_YAW_TOPIC, &pubmessage, &token);
    		printf("Waiting for up to %d seconds for publication of %s\n"
            		"on topic %s for client with ClientID: %s\n",
            		(int)(MQ_TIMEOUT/1000), tmp_msg, MQ_YAW_TOPIC, MQ_ID);
    		rc = MQTTClient_waitForCompletion(mqClient, token, MQ_TIMEOUT);
    		printf("Message with delivery token %d delivered\n", token);

  		sprintf(tmp_msg, "Temperature: %d", temp);
    		pubmessage.payload = tmp_msg;
    		pubmessage.payloadlen = strlen(tmp_msg);
    		pubmessage.qos = MQ_QOS;
    		pubmessage.retained = 0;
    		MQTTClient_publishMessage(mqClient, MQ_TEMP_TOPIC, &pubmessage, &token);
    		printf("Waiting for up to %d seconds for publication of %s\n"
            		"on topic %s for client with ClientID: %s\n",
            		(int)(MQ_TIMEOUT/1000), tmp_msg, MQ_TEMP_TOPIC, MQ_ID);
    		rc = MQTTClient_waitForCompletion(mqClient, token, MQ_TIMEOUT);
    		printf("Message with delivery token %d delivered\n", token);

    		MQTTClient_disconnect(mqClient, 10000);
    		MQTTClient_destroy(&mqClient);
		    break;
	     case OPTION_MODE_COUCH :


	     default :
                printf ("temp: %4d, pitch: %4.0f, roll: %4.0f, yaw: %4.0f\n",
                  temp, angles1.x, angles1.y, angles1.z);
                break;
	  }
      }
      angles2.x = angles1.x;
      angles2.y = angles1.y;
      angles2.z = angles1.z;
    }
  }
  return 0;
}
