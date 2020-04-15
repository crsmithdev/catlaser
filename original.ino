#include <Servo.h>

const double TO_DEGREES = 180.0 / M_PI;
const double RANGE_CONVERT = 2000.0 / 180.0;
const double RANGE_OFFSET = 500;
const double THETA_OFFSET = -500;
const double PHI_OFFSET = 500;
const double PHI_CORRECT = 75;
const double THETA_CORRECT = 0;
const double CAMERA_Z = 62;
const double X_MIN = 0;
const double X_MAX = 100;
const double Y_MIN = 0;
const double Y_MAX = 50;
const double SPEED = 2;
const double DELAY = 10;

Servo servo_theta;
Servo servo_phi;

struct Point
{
  double x;
  double y;
};

void setup()
{
  Serial.begin(9600);

  pinMode(8, OUTPUT);
  servo_phi.attach(9);
  servo_theta.attach(10);

  digitalWrite(8, LOW);

  Point origin = {x : 0, y : 0};
  move(origin);
}

void log(double x, double y, double dx, double dy, double dz,
         double theta, double phi, double theta_ms, double phi_ms)
{
  Serial.print("x: ");
  Serial.print(x);
  Serial.print(" y: ");
  Serial.print(y);
  Serial.print(" dx: ");
  Serial.print(dx);
  Serial.print(" dy: ");
  Serial.print(dy);
  Serial.print(" dz: ");
  Serial.print(dz);
  Serial.print(" theta: ");
  Serial.print(theta);
  Serial.print(" phi: ");
  Serial.print(phi);
  Serial.print(" theta_ms: ");
  Serial.print(theta_ms);
  Serial.print(" phi_ms: ");
  Serial.print(phi_ms);
  Serial.print("\n");
}

void move(Point to)
{
  double dx = to.x;
  double dy = to.y;
  double dz = -CAMERA_Z;
  double phi = atan2(dy, dx) * TO_DEGREES;
  double theta = atan2(sqrt(sq(dx) + sq(dy)), dz) * TO_DEGREES;
  double phi_ms = phi * RANGE_CONVERT + RANGE_OFFSET;

  double theta_ms = theta * RANGE_CONVERT + RANGE_OFFSET;
  log(to.x, to.y, dx, dy, dz, theta, phi, theta_ms, phi_ms);

  servo_theta.writeMicroseconds(theta_ms + THETA_OFFSET + THETA_CORRECT);
  servo_phi.writeMicroseconds(phi_ms + PHI_OFFSET + PHI_CORRECT);
}

void traverse(Point from, Point to)
{
  int distance = sqrt(sq(to.x - from.x) + sq(to.y - from.y));
  int steps = distance / SPEED;
  double x_step = (to.x - from.x) / steps;
  double y_step = (to.y - from.y) / steps;
  Point current = {x : from.x, y : from.y};

  Serial.print("from.x: ");
  Serial.print(from.x);
  Serial.print(" from.y: ");
  Serial.print(from.y);
  Serial.print(" to.x: ");
  Serial.print(to.x);
  Serial.print(" to.y: ");
  Serial.print(to.y);
  Serial.print(" distance: ");
  Serial.print(distance);
  Serial.print(" steps: ");
  Serial.print(steps);
  Serial.print(" x_step: ");
  Serial.print(x_step);
  Serial.print(" y_step: ");
  Serial.print(y_step);
  Serial.print("\n");

  int i = 0;
  do
  //for (int i = 0; i < steps; i++)
  {
    Serial.print(current.x);
    Serial.print(", ");
    Serial.print(current.y);
    Serial.print("\n");

    move(current);
    current.x += x_step;
    current.y += y_step;
    delay(DELAY);
    i++;
  } while (i < steps);

  Serial.print("\n");
}

void loop()
{
  digitalWrite(8, HIGH);
  // servo_phi.writeMicroseconds(1500);
  // servo_theta.writeMicroseconds(2000);

  Point p1 = {x : X_MIN, y : Y_MIN};
  Point p2 = {x : p1.x, y : Y_MAX};
  Point p3 = {x : X_MAX, y : p2.y};
  Point p4 = {x : p3.x, y : Y_MIN};

  // p1 = {x : 5, y : 5};
  // p2 = {x : 5, y : 35};
  // p3 = {x : 35, y : 35};
  // p4 = {x : 35, y : 5};

  traverse(p1, p2);
  traverse(p2, p3);
  traverse(p3, p4);
  traverse(p4, p1);

  // for (int i = 0; i < 800; i++)
  // {
  //   servo_theta.writeMicroseconds(1500 - i);
  //   Serial.println(1500 - i);
  //   delay(50);
  // }

  delay(1000);
}
