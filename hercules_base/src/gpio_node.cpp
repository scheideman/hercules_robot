#include <fcntl.h>
#include <unistd.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>

#define SYSFS_GPIO_DIR "/sys/class/gpio"
#define MAX_BUF 64

// gpioGetValue
// Get the value of the requested GPIO pin ; value return is 0 or 1
// Return: Success = 0 ; otherwise open file error
int gpioGetValue(int gpio, unsigned char* value) {
  int fileDescriptor;
  char commandBuffer[MAX_BUF];
  char ch;

  snprintf(commandBuffer, sizeof(commandBuffer), SYSFS_GPIO_DIR "/gpio%d/value",
           gpio);

  fileDescriptor = open(commandBuffer, O_RDONLY);
  if (fileDescriptor < 0) {
    char errorBuffer[128];
    snprintf(errorBuffer, sizeof(errorBuffer),
             "gpioGetValue unable to open gpio%d", gpio);
    perror(errorBuffer);
    return fileDescriptor;
  }

  if (read(fileDescriptor, &ch, 1) != 1) {
    perror("gpioGetValue");
    return fileDescriptor;
  }

  if (ch != '0') {
    *value = 1;
  } else {
    *value = 0;
  }

  close(fileDescriptor);
  return 0;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "gpio_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh;

  // read params
  int start_btn_gpio = 186;
  int rate = 10;
  std::string mission_start_topic = "/mission_start";
  private_nh.getParam("start_btn_gpio", start_btn_gpio);
  private_nh.getParam("rate", rate);
  nh.getParam("topic/mission_start", mission_start_topic);
  std::cout << "GPIO NODE PARAMS" << std::endl;
  std::cout << "=================" << std::endl;
  std::cout << "start_btn_gpio: " << start_btn_gpio << std::endl;
  std::cout << "output rate: " << rate << std::endl;
  std::cout << "mission start topic: " << mission_start_topic << std::endl;
  std::cout << "=================" << std::endl;

  ros::Publisher start_pub =
      nh.advertise<std_msgs::Bool>(mission_start_topic, 4);

  ros::Rate r(rate);
  unsigned char mission_start = 0;
  std_msgs::Bool start_button;

  while (ros::ok()) {
    gpioGetValue(start_btn_gpio, &mission_start);
    std::cout << mission_start << std::endl;
    start_button.data = static_cast<unsigned char>(not mission_start);
    start_pub.publish(start_button);

    ros::spinOnce();
    r.sleep();
  }
  return 0;
}
