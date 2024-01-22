#include <GLFW/glfw3.h>
#include <vector>
#include <iostream>

#define M_PI 3.14159

struct Vector2D {
    float x, y;

    Vector2D(float x = 0.0f, float y = 0.0f) : x(x), y(y) {}

    Vector2D operator+(const Vector2D& other) const {
        return Vector2D(x + other.x, y + other.y);
    }

    Vector2D& operator+=(const Vector2D& other) {
        x += other.x;
        y += other.y;
        return *this;
    }

    Vector2D& operator-=(const Vector2D& other) {
        x -= other.x;
        y -= other.y;
        return *this;
    }

    Vector2D& operator*=(const Vector2D& other) {
        x *= other.x;
        y *= other.y;
        return *this;
    }

    Vector2D operator*(float scalar) const {
        return Vector2D(x * scalar, y * scalar);
    }

    Vector2D operator-(const Vector2D& other) const {
        return Vector2D(x - other.x, y - other.y);
    }

};

struct Color {
    float r, g, b;

    Color(float r = 0.0f, float g = 0.0f, float b = 0.0f) : r(r), g(g), b(b) {}
};

class Circle {
public:
    float radius;
    Color color; // Represented as RGB values between 0 and 1
    float mass;
    Vector2D position;
    Vector2D velocity;
    float angularVelocity;
    Vector2D acceleration;
    float angularAcceleration;
    std::vector<Vector2D> forces;
    std::vector<Vector2D> trail;
    int trailLimit;

    Circle(float radius, Color color, float mass, Vector2D position, Vector2D velocity, int trailLimit = -1)
        : radius(radius), color(color), mass(mass), position(position), velocity(velocity),
        angularVelocity(0), acceleration(0, 0), angularAcceleration(0), trailLimit(trailLimit) {}

    void drawTrail() {
        glPointSize(3.0f); // Set point size
        glColor3f(color.r * 0.5, color.g * 0.5, color.b * 0.5); // Dimmed color for the trail
        glBegin(GL_POINTS);
        for (const auto& point : trail) {
            glVertex2f(point.x, point.y);
        }
        glEnd();
    }

    void applyForce(const Vector2D& force) {
        forces.push_back(force);
    }

    void draw() {
        drawTrail(); // Draw the trail first

        const int segments = 18; // Number of segments to approximate a circle
        float angleStep = 2.0f * M_PI / segments;

        glColor3f(color.r, color.g, color.b);
        glBegin(GL_LINE_LOOP);
        for (int i = 0; i < segments; i++) {
            float angle = angleStep * i;
            float x = radius * cosf(angle) + position.x;
            float y = radius * sinf(angle) + position.y;
            glVertex2f(x, y);
        }
        glEnd();
    }

    void update(float deltaTime) {
        updatePhysics(deltaTime);
        updateTrail();
    }

private:
    void updateTrail() {
        trail.push_back(position);
        if (trailLimit != -1 && trail.size() > trailLimit) {
            trail.erase(trail.begin());
        }
    }

    void updatePhysics(float deltaTime) {
        Vector2D resultantForce(0, 0);
        for (const auto& force : forces) {
            resultantForce += force;
        }
        forces.clear();

        acceleration = resultantForce * (1.0f / mass);
        velocity += acceleration * deltaTime;
        position += velocity * deltaTime;

        // Wall collision
        if (position.x < radius || position.x > 1000 - radius) {
            velocity.x = -velocity.x;
        }
        if (position.y < radius || position.y > 1000 - radius) {
            velocity.y = -velocity.y;
        }
    }
};

std::vector<Circle> circles;
float maxGravitationalForce = 100.0f;
float restitution = 0.8f;
const float timeStep = 1.0f / 60.0f; // 60 updates per second
float accumulator = 0.0f;

Vector2D calculateGravitationalForce(const Circle& a, const Circle& b) {
    const float G = 6.67; // Adjusted gravitational constant for visibility
    Vector2D distanceVector = b.position - a.position;
    float distance = sqrtf(distanceVector.x * distanceVector.x + distanceVector.y * distanceVector.y);

    // Minimum distance threshold to avoid infinite force
    float minDistance = 20.0f;

    // Use max between actual distance and minimum distance to avoid very high forces
    distance = std::max(distance, minDistance);

    distance /= 75; 

    float forceMagnitude = G * (a.mass * b.mass) / (distance * distance);
    forceMagnitude = std::min(forceMagnitude, maxGravitationalForce); // Cap the force

    // Normalize distance vector and multiply by force magnitude
    return Vector2D(distanceVector.x / distance * forceMagnitude, distanceVector.y / distance * forceMagnitude);
}

bool checkCollision(const Circle& a, const Circle& b) {
    Vector2D delta = a.position - b.position;
    float distance = sqrtf(delta.x * delta.x + delta.y * delta.y);
    return distance <= (a.radius + b.radius);
}

static void handleCollision(Circle& a, Circle& b) {
    Vector2D delta = b.position - a.position;
    float distance = sqrtf(delta.x * delta.x + delta.y * delta.y);
    if (distance == 0.0f) {
        // To avoid division by zero
        return;
    }

    // Normal and Tangential Vectors
    Vector2D normal = delta * (1.0f / distance);
    Vector2D tangent(-normal.y, normal.x);

    // Dot Product Tangential
    float dotProductTangentA = a.velocity.x * tangent.x + a.velocity.y * tangent.y;
    float dotProductTangentB = b.velocity.x * tangent.x + b.velocity.y * tangent.y;

    // Dot Product Normal
    float dotProductNormalA = a.velocity.x * normal.x + a.velocity.y * normal.y;
    float dotProductNormalB = b.velocity.x * normal.x + b.velocity.y * normal.y;

    // Conservation of momentum in 1D for normal component
    float m1 = (dotProductNormalA * (a.mass - b.mass) + 2 * b.mass * dotProductNormalB) / (a.mass + b.mass);
    float m2 = (dotProductNormalB * (b.mass - a.mass) + 2 * a.mass * dotProductNormalA) / (a.mass + b.mass);

    // Apply the restitution coefficient
    m1 *= restitution;
    m2 *= restitution;

    // Update velocities
    a.velocity.x = tangent.x * dotProductTangentA + normal.x * m1;
    a.velocity.y = tangent.y * dotProductTangentA + normal.y * m1;
    b.velocity.x = tangent.x * dotProductTangentB + normal.x * m2;
    b.velocity.y = tangent.y * dotProductTangentB + normal.y * m2;

    // Position correction with a small buffer to prevent sticking
    float penetrationDepth = (a.radius + b.radius - distance) + 0.1f;  // Adding a small buffer
    Vector2D correction = normal * (penetrationDepth / 2.0f);
    a.position -= correction;
    b.position += correction;
}


int main(void)
{
    GLFWwindow* window;

    /* Initialize the library */
    if (!glfwInit())
        return -1;

    /* Create a windowed mode window and its OpenGL context */
    window = glfwCreateWindow(1000, 1000, "Physics Simulator", NULL, NULL);

    if (!window)
    {
        glfwTerminate();
        return -1;
    }

    /* Make the window's context current */
    glfwMakeContextCurrent(window);

    // Set up orthographic projection
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glOrtho(0, 1000, 1000, 0, -1, 1); // Set coordinate system from (0,0) to (1000,1000)
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glClearColor(0.0f, 0.0f, 0.0f, 1.0f); // Set clear color to black

    // Create a circle instance
    // radius, color, mass, position
    circles.push_back(Circle(100.0f, Color(1.0f, 0.0f, 0.0f), 50.0f, Vector2D(500.0f, 750.0f), Vector2D(50.0f, -50.0f), -1));
    circles.push_back(Circle(50.0f, Color(0.0f, 0.0f, 1.0f), 25.0f, Vector2D(200.0f, 300.0f), Vector2D(25.0f, 50.0f), -1));
    //circles.push_back(Circle(5.0f, Color(0.0f, 1.0f, 0.0f), 0.4f, Vector2D(700.0f, 800.0f), Vector2D(30.0f, -50.0f), -1));


    double lastTime = glfwGetTime();

    bool enable_gravity = true;

    // Main loop
    //
    while (!glfwWindowShouldClose(window))
    {
        double currentTime = glfwGetTime();
        float deltaTime = static_cast<float>(currentTime - lastTime);
        lastTime = currentTime;


        if (enable_gravity) {
            // Apply gravitational forces and update all circles
            for (size_t i = 0; i < circles.size(); ++i) {
                Vector2D totalForce(0.0f, 0.0f);
                for (size_t j = 0; j < circles.size(); ++j) {
                    if (i != j) {
                        totalForce += calculateGravitationalForce(circles[i], circles[j]);
                    }
                }
                circles[i].applyForce(totalForce);
            }
        }

        // Update physics for all circles
        for (auto& circle : circles) {
            circle.update(deltaTime);
        }

        // Check for collision
        for (size_t i = 0; i < circles.size(); ++i) {
            for (size_t j = i + 1; j < circles.size(); ++j) {
                if (checkCollision(circles[i], circles[j])) {
                    handleCollision(circles[i], circles[j]);
                }
            }
        }


        // Render
        glClear(GL_COLOR_BUFFER_BIT);
        for (auto& circle : circles) {
            circle.draw();
        }
        glfwSwapBuffers(window);

        glfwPollEvents();
    }


    glfwTerminate();
    return 0;
}