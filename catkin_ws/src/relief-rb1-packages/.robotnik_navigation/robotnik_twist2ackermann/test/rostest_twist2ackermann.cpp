#include <gtest/gtest.h>

#include <ros/ros.h>

#include <geometry_msgs/Twist.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include <robotnik_twist2ackermann/Twist2Ackermann.h>
// TODO: check dependencies between packages

class Twist2AckTest : public ::testing::Test
{
public:
  void ackermannCmdCallback(const ackermann_msgs::AckermannDriveStampedConstPtr& ack)
  {
    number_of_ack_cmd_received++;
    ack_cmd_received = ack->drive;
  }

protected:
  virtual void SetUp()
  {
    component = new Twist2Ackermann(nh);

    twist_cmd_pub = nh.advertise<geometry_msgs::Twist>(component->twist_cmd_topic_, 1);
    ack_status_pub = nh.advertise<ackermann_msgs::AckermannDriveStamped>(component->ack_status_topic_, 1);

    ack_cmd_sub =
        nh.subscribe(component->ack_cmd_topic_, 1, &Twist2AckTest::ackermannCmdCallback, (Twist2AckTest*)this);

    ack_status_timer = nh.createWallTimer(ros::WallDuration(0.01), &Twist2AckTest::ackermannStatusTimerCallback, this);

    number_of_ack_cmd_received = 0;
    ack_status = ackermann_msgs::AckermannDrive();
    k_steering = 0.1;
    k_speed = 0.3;

    component->asyncStart();
  }

  virtual void TearDown()
  {
    component->stop();
    delete component;
  }

  virtual void ackermannStatusTimerCallback(const ros::WallTimerEvent& event)
  {
    ack_status.steering_angle += (ack_cmd_received.steering_angle - ack_status.steering_angle) * k_steering;
    ack_status.speed += (ack_cmd_received.speed - ack_status.speed) * k_speed;

    ackermann_msgs::AckermannDriveStamped ack_stamped;
    ack_stamped.drive = ack_status;
    ack_stamped.header.stamp = ros::Time::now();
    ack_status_pub.publish(ack_stamped);
  }

  bool waitForComponentAtReadyState(ros::Duration max_wait_time = ros::Duration(5))
  {
    ros::Time wait(ros::Time::now() + max_wait_time);
    while (ros::Time::now() < wait)
    {
      if (component->getState() == robotnik_msgs::State::READY_STATE)
        break;

      ros::WallDuration(0.01).sleep();
      ros::spinOnce();
    }
    if (component->getState() == robotnik_msgs::State::READY_STATE)
      return true;
    return false;
  }

  ros::NodeHandle nh;
  Twist2Ackermann* component;
  ros::Publisher twist_cmd_pub;
  ros::Publisher ack_status_pub;
  ros::Subscriber ack_cmd_sub;

  ros::WallTimer ack_status_timer;

  ackermann_msgs::AckermannDrive ack_cmd_received;
  ackermann_msgs::AckermannDrive ack_status;
  double k_steering;
  double k_speed;

  int number_of_ack_cmd_received;
};

TEST_F(Twist2AckTest, shouldSubscribeToCmdVel)
{
  // GIVEN a twist2ackermann component and cmd publisher
  ros::Publisher twist_cmd_pub = nh.advertise<geometry_msgs::Twist>(component->twist_cmd_topic_, 1);

  // WHEN the component gets ready
  //  component->asyncStart();
  waitForComponentAtReadyState();

  // THEN tries to subscribe to the topic specified
  EXPECT_EQ(1, twist_cmd_pub.getNumSubscribers());
}

TEST_F(Twist2AckTest, shouldAdvertiseAnAckTopic)
{
  // GIVEN a twist2ackermann component and an ack subscriber
  ros::Subscriber ack_cmd_sub =
      nh.subscribe(component->ack_cmd_topic_, 1, &Twist2AckTest::ackermannCmdCallback, (Twist2AckTest*)this);

  // WHEN the component gets ready starts
  //  component->asyncStart();
  EXPECT_TRUE(waitForComponentAtReadyState());

  // THEN an AckermannDriveStamped topic is advertised
  EXPECT_EQ(1, ack_cmd_sub.getNumPublishers());
}

TEST_F(Twist2AckTest, shouldOnlyAccetPositiveWheelbaseValues)
{
  // GIVEN a twist2ackermann component (already created in the fixture)

  // WHEN we set a negative wheelbase or turning radius
  bool accepts_wheelbase = false;
  accepts_wheelbase = component->setWheelbase(-1);

  bool accepts_turning_radius = false;
  accepts_turning_radius = component->setMinimumTurningRadius(-1);

  // THEN the component does not accept it
  EXPECT_FALSE(accepts_wheelbase);
  EXPECT_FALSE(accepts_turning_radius);
}

TEST_F(Twist2AckTest, shouldTransformCommandsWithOnlyLinearSpeed)
{
  // GIVEN a twist2ackermann component with a certain wheelbase, and a twist command
  double wheelbase = 2.2;
  component->setWheelbase(wheelbase);

  geometry_msgs::Twist twist;
  twist.linear.x = 2;

  ackermann_msgs::AckermannDrive correct_ack;
  correct_ack.speed = 2;

  // WHEN the component tries to convert a valid command
  ackermann_msgs::AckermannDrive ack_computed;
  bool transform_ok = false;
  transform_ok = component->transformTwistToAckermann(twist, ack_computed);

  // THEN the correct command is computed
  EXPECT_TRUE(transform_ok);
  EXPECT_EQ(correct_ack.steering_angle, ack_computed.steering_angle);
  EXPECT_EQ(correct_ack.steering_angle_velocity, ack_computed.steering_angle_velocity);
  EXPECT_EQ(correct_ack.speed, ack_computed.speed);
  EXPECT_EQ(correct_ack.acceleration, ack_computed.acceleration);
  EXPECT_EQ(correct_ack.jerk, ack_computed.jerk);
}

TEST_F(Twist2AckTest, shouldTransformCommandsWithLinearAndAngularSpeedThatAreAboveTheMinimumTurningRadius)
{
  // GIVEN a twist2ackermann component with a certain wheelbase and turning radius and a twist command
  double wheelbase = 2.2;
  component->setWheelbase(wheelbase);

  double turning_radius = 1.1;
  component->setMinimumTurningRadius(turning_radius);

  geometry_msgs::Twist twist;
  twist.linear.x = 2;

  ackermann_msgs::AckermannDrive correct_ack;
  correct_ack.speed = 2;

  // WHEN the component tries to convert a valid command
  ackermann_msgs::AckermannDrive ack_computed;
  bool transform_ok = false;
  transform_ok = component->transformTwistToAckermann(twist, ack_computed);

  // THEN the correct command is computed
  EXPECT_TRUE(transform_ok);
  EXPECT_EQ(correct_ack.steering_angle, ack_computed.steering_angle);
  EXPECT_EQ(correct_ack.steering_angle_velocity, ack_computed.steering_angle_velocity);
  EXPECT_EQ(correct_ack.speed, ack_computed.speed);
  EXPECT_EQ(correct_ack.acceleration, ack_computed.acceleration);
  EXPECT_EQ(correct_ack.jerk, ack_computed.jerk);
}

TEST_F(Twist2AckTest, shouldNOTTransformCommandsThatAreBelowTheMinimumTurningRadius)
{
  // GIVEN a twist2ackermann component with a certain wheelbase and turning radius, and a twist command
  double wheelbase = 2.2;
  component->setWheelbase(wheelbase);

  double turning_radius = 1.1;
  component->setMinimumTurningRadius(turning_radius);

  geometry_msgs::Twist twist;
  twist.linear.x = 0.01;
  twist.angular.z = 2;

  // WHEN the component tries to convert an invalid command
  ackermann_msgs::AckermannDrive ack_computed;
  bool transform_ok = false;
  transform_ok = component->transformTwistToAckermann(twist, ack_computed);

  // THEN it cannot convert the command
  EXPECT_FALSE(transform_ok);
}

TEST_F(Twist2AckTest, shouldNotPublishAckCmdIfTwistCmdIsNotSent)
{
  // GIVEN a twist2ackermann component

  // WHEN no twist commands are issued
  ros::Duration(1).sleep();

  // THEN no ackermann commands are sent
  EXPECT_EQ(0, number_of_ack_cmd_received);
}

TEST_F(Twist2AckTest, shouldPublishSpeed0UntilSteeringAngleIsCorrectWhenRobotIsStopped)
{
  // GIVEN a robot with the driving steer at a certain position but stopped
  double wheelbase = 2.2;
  component->setWheelbase(wheelbase);

  double turning_radius = 1.1;
  component->setMinimumTurningRadius(turning_radius);

  ack_status.steering_angle = 0.0;
  ack_status.speed = 0.0;

  // WHEN a command with a different steering angle is issued
  geometry_msgs::Twist twist;
  twist.linear.x = 1.0;
  twist.angular.z = 0.5;

  ackermann_msgs::AckermannDrive ack;
  ASSERT_TRUE(component->transformTwistToAckermann(twist, ack));  // we fail here if conversion cannot be done

  // THEN a command with no speed but the correct steering angle is published
  ros::Rate r(10);
  ros::Time wait_time = ros::Time::now() + ros::Duration(5);  // 1 + std::abs(ack.steering_angle -
  // ack_status.steering_angle) / k_steering);
  twist_cmd_pub.publish(twist);
  while (ros::Time::now() < wait_time)
  {
    r.sleep();
    if (number_of_ack_cmd_received < 0)
      continue;

    if (component->inPosition(ack) == true)
      break;
    EXPECT_EQ(0, ack_cmd_received.speed);
    EXPECT_EQ(ack.steering_angle, ack_cmd_received.steering_angle);
    twist_cmd_pub.publish(twist);
  }

  wait_time = ros::Time::now() + ros::Duration(5);
  while (ros::Time::now() < wait_time)
  {
    twist_cmd_pub.publish(twist);
    if (ack_cmd_received.speed != 0)
      break;
    r.sleep();
  }

  EXPECT_TRUE(component->inPosition(ack));
  EXPECT_EQ(ack.speed, ack_cmd_received.speed);
  EXPECT_EQ(ack.steering_angle, ack_cmd_received.steering_angle);
}

///////TEST_F(Twist2AckTest, shouldPublishSpeed0UntilSteeringAngleIsCorrectWhenRobotIsMoving
///////{
///////  // GIVEN
///////
///////  // WHEN
///////
///////  // THEN
///////}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rosunit_procedure_component");

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
