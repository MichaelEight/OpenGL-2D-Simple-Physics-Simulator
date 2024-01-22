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

Vector2D calculateGravitationalForce(const Circle& a, const Circle& b) {
    const float G = 6.67; // Adjusted gravitational constant for visibility
    Vector2D distanceVector = b.position - a.position;
    float distance = sqrtf(distanceVector.x * distanceVector.x + distanceVector.y * distanceVector.y);

    // Minimum distance threshold to avoid infinite force
    float minDistance = 20.0f; // You can adjust this value

    // Use max between actual distance and minimum distance to avoid very high forces
    distance = std::max(distance, minDistance);

    distance /= 75; 

    float forceMagnitude = G * (a.mass * b.mass) / (distance * distance);

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
    Vector2D normal = delta * (1.0f / sqrtf(delta.x * delta.x + delta.y * delta.y));

    float v1n = normal.x * a.velocity.x + normal.y * a.velocity.y;
    float v2n = normal.x * b.velocity.x + normal.y * b.velocity.y;

    float v1nTag = (v1n * (a.mass - b.mass) + 2 * b.mass * v2n) / (a.mass + b.mass);
    float v2nTag = (v2n * (b.mass - a.mass) + 2 * a.mass * v1n) / (a.mass + b.mass);

    a.velocity += normal * (v1nTag - v1n);
    b.velocity += normal * (v2nTag - v2n);
}

std::vector<Circle> circles;

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
    circles.push_back(Circle(50.0f, Color(1.0f, 0.0f, 0.0f), 1.0f, Vector2D(500.0f, 500.0f), Vector2D(0.0f, 50.0f), -1));
    circles.push_back(Circle(50.0f, Color(0.0f, 0.0f, 1.0f), 1.0f, Vector2D(300.0f, 300.0f), Vector2D(0.0f, -50.0f), -1));


    double lastTime = glfwGetTime();

    // Main loop
    //
    while (!glfwWindowShouldClose(window))
    {
        double currentTime = glfwGetTime();
        float deltaTime = static_cast<float>(currentTime - lastTime);
        lastTime = currentTime;

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
        
        for (size_t i = 0; i < circles.size(); ++i) {
            circles[i].update(deltaTime);
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