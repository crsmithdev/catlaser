#include <Servo.h>

const double RADIANS_TO_DEGREES = 180.0 / M_PI;
const double DEGREES_TO_MS = 2000.0 / 180.0;
const double MS_MIN = 500;
const double THETA_OFFSET = -500;
const double THETA_CORRECT = 0;
const double PHI_OFFSET = 500;
const double PHI_CORRECT = 75;

const double X_MIN = 0;
const double X_MAX = 100;
const double Y_MIN = 0;
const double Y_MAX = 50;
const double Z_OFFSET = 62;

const double RESOLUTION = .1;
const double SPEED_BASE = 20.0;
const double SPEED_VARIANCE = 10.0;

const double CIRCLE_MIN_RADIUS = 3.0;
const double CIRCLE_MAX_RADIUS = 8.0;
const int CIRCLE_MIN_REPEAT = 2;
const int CIRCLE_MAX_REPEAT = 5;
const int CIRCLE_STEPS = 20;

const int SCATTER_MIN_POINTS = 3;
const int SCATTER_MAX_POINTS = 10;
const int SCATTER_MIN_RADIUS = 3.0;
const int SCATTER_MAX_RADIUS = 8.0;
const int SCATTER_MIN_DELAY = 20;
const int SCATTER_MAX_DELAY = 150;

const int TEASE_MIN_DELAY = 100;
const int TEASE_MAX_DELAY = 2000;

const int PIN_LASER = 8;
const int PIN_PHI = 9;
const int PIN_THETA = 10;

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

  pinMode(PIN_LASER, OUTPUT);
  servo_phi.attach(PIN_PHI);
  servo_theta.attach(PIN_THETA);

  randomSeed(analogRead(0));
  digitalWrite(8, HIGH);
}

void log_move(double x, double y, double dx, double dy, double dz,
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

void log_traverse(Point from, Point to, double distance, int steps,
                  double duration, double interval, double x_step, double y_step)
{
  Serial.print("traverse -> from.x: ");
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
  Serial.print(" duration: ");
  Serial.print(duration);
  Serial.print(" interval: ");
  Serial.print(interval);
  Serial.print(" x_step: ");
  Serial.print(x_step);
  Serial.print(" y_step: ");
  Serial.print(y_step);
  Serial.print("\n");
}

void move(Point to)
{
  to.x = min(max(to.x, X_MIN), X_MAX);
  to.y = min(max(to.y, Y_MIN), Y_MAX);

  double dx = to.x;
  double dy = to.y;
  double dz = -Z_OFFSET;
  double phi = atan2(dy, dx) * RADIANS_TO_DEGREES;
  double theta = atan2(sqrt(sq(dx) + sq(dy)), dz) * RADIANS_TO_DEGREES;
  double phi_ms = phi * DEGREES_TO_MS + MS_MIN;
  double theta_ms = theta * DEGREES_TO_MS + MS_MIN;

  // log_move(to.x, to.y, dx, dy, dz, theta, phi, theta_ms, phi_ms);

  servo_theta.writeMicroseconds(theta_ms + THETA_OFFSET + THETA_CORRECT);
  servo_phi.writeMicroseconds(phi_ms + PHI_OFFSET + PHI_CORRECT);
}

void traverse(Point from, Point to, double speed)
{
  int distance = sqrt(sq(to.x - from.x) + sq(to.y - from.y));
  int steps = distance / RESOLUTION;
  double duration = distance / speed * 1000;
  double interval = duration / steps;
  double x_step = (to.x - from.x) / steps;
  double y_step = (to.y - from.y) / steps;
  Point current = {x : from.x, y : from.y};
  int i = 0;

  // log_traverse(from, to, distance, steps, duration, interval, x_step, y_step);

  do
  {
    move(current);
    current.x += x_step;
    current.y += y_step;
    delay(interval);
    i++;
  } while (i < steps);
}

void scatter(Point center, double radius, int points, int speed)
{
  for (int i = 0; i < points; i++)
  {
    double radius = random(1, radius);
    Point next = {
      x : center.x + cos(random(0, 180)) * radius,
      y : center.y + sin(random(0, 180)) * radius,
    };

    traverse(center, next, speed);
    delay(random(SCATTER_MIN_DELAY, SCATTER_MAX_DELAY));
    traverse(next, center, speed);
    delay(random(SCATTER_MIN_DELAY, SCATTER_MAX_DELAY));
  }
}

void circle(Point center, double radius, int repeat, int steps, int speed)
{
  double step_angle = 2 * M_PI / steps;
  Point current = center;

  for (int i = 0; i < repeat; i++)
  {
    Point next = {
      x : center.x + cos(0) * radius,
      y : center.y + sin(0) * radius,
    };

    traverse(current, next, speed);

    for (int j = 1; j < steps; j++)
    {
      next = {
        x : center.x + cos(step_angle * j) * radius,
        y : center.y + sin(step_angle * j) * radius,
      };
      traverse(current, next, speed);
      current = next;
    }
  }

  traverse(current, center, speed);
}

void tease()
{
  Point p1 = {x : random(X_MIN, X_MAX), y : random(Y_MIN, Y_MAX)};
  Point p2 = {x : random(X_MIN, X_MAX), y : random(Y_MIN, Y_MAX)};

  while (true)
  {

    double variance = 0 - (SPEED_VARIANCE / 2) + random(SPEED_VARIANCE);
    double speed = SPEED_BASE + variance;

    traverse(p1, p2, speed);

    if (random(0, 2) == 1)
    {
      double radius = random(CIRCLE_MIN_RADIUS, CIRCLE_MAX_RADIUS);
      double times = random(CIRCLE_MIN_REPEAT, CIRCLE_MAX_REPEAT);
      circle(p2, radius, times, CIRCLE_STEPS, speed);
    }
    else
    {
      double points = random(SCATTER_MIN_POINTS, SCATTER_MAX_POINTS);
      double radius = random(SCATTER_MIN_RADIUS, SCATTER_MAX_RADIUS);
      scatter(p2, radius, points, speed);
    }

    p1 = p2;
    p2 = {x : random(X_MIN, X_MAX), y : random(Y_MIN, Y_MAX)};

    delay(random(TEASE_MIN_DELAY, TEASE_MIN_DELAY));
  }
}

void loop()
{
  tease();
}
