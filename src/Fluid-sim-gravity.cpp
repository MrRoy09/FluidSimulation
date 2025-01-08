#include <iostream>
#include "SFML/Graphics.hpp"
#include <vector>
#include <cmath>
#include <random>
#include <map>
#include <thread>
#include <chrono>
#include <mutex>
#include <list>

// to-do fix crashes related to density =0
double delta_t = 0.002;
double gravity;
int screen_width = 1280;
int screen_height = 800;
bool gravity_on = 1;
float target_density = 60; // target density when gravity on = 60
double pressureMultiplier = 40000000; // set to 40000000 when gravity on 20000000 otherwise
float damping_factor = 0.9;
float smoothing_radius = 80;
int num_particles;
int number_threads;

float impact_radius_mouse = 200;
float impact_radius_mouse2 =200;
float strength = 2000; // set to 2000 when gravity on
float strength2 = 2000; // set to 2000 when gravity on


sf::Color blue = { 2,8,240};
sf::Color red = { 237,2,2};
sf::Color green = { 37,126,69};

static float magnitude(sf::Vector2f vector) {
    return sqrt(pow(vector.x, 2) + pow(vector.y, 2));
}

void map_average_max(std::map<int, float> param) {
   std::map<int,float>::iterator result = std::max_element(param.begin(), param.end());
   std::cout << result->second<< "\n";
}

class particle {
public:
    sf::Vector2f position;
    sf::Vector2f velocity;
    sf::CircleShape particle_shape;
    float particle_radius = 3;
    float mass = 1000;
    int id;

    particle(int id,float pos_x,float pos_y,float vel_x, float vel_y) {
        this->id = id;
        this->position.x = pos_x;
        this->position.y = pos_y;
        this->velocity.x = vel_x;
        this->velocity.y = vel_y;
        this->particle_shape.setRadius(particle_radius);
        this->particle_shape.setPosition(position);
        this->particle_shape.setFillColor(sf::Color::Blue);
    }

    void render(sf::RenderWindow &wind) {
        this->particle_shape.setPosition(this->position-sf::Vector2f(particle_radius,particle_radius));
        wind.draw(this->particle_shape);
       
    }

    void boundary_check() {
        if (this->position.x <= 0) {
            this->position.x = 0+particle_shape.getRadius();
            this->velocity.x = -1 * this->velocity.x*damping_factor;
        }
        else if (this->position.x >= screen_width) {
            this->position.x = screen_width-particle_shape.getRadius();
            this->velocity.x = -1 * this->velocity.x * damping_factor;
        }
        if (this->position.y <= 0) {
            this->position.y = 0+particle_shape.getRadius();
            this->velocity.y = -1 * this->velocity.y*damping_factor;
        }

        else if (this->position.y >= screen_height) {
            this->position.y = screen_height-particle_shape.getRadius();
            this->velocity.y = -1 * this->velocity.y * damping_factor;
        }
    }


    void update(sf::Vector2f pressure_force) {
        boundary_check();
        if (gravity_on) {
            gravity = 500000;
        }
        else {
            gravity = 0;
        }
        this->velocity.y = pressure_force.y * delta_t+this->velocity.y*0.92; // set to 0.92 when gravity on
        this->velocity.y += gravity * delta_t;
        this->velocity.x = pressure_force.x * delta_t + this->velocity.x * 0.92;
        this->position.x += this->velocity.x * delta_t;
        this->position.y += this->velocity.y * delta_t;
        float magnitude_velocity = magnitude(this->velocity);
        //std::cout << magnitude_velocity << "\n";
        if (magnitude_velocity < 4000) {
            this->particle_shape.setFillColor(blue);
        }
        else if (magnitude_velocity > 4000 && magnitude_velocity < 7000) {
            this->particle_shape.setFillColor(green);
        }
        else if (magnitude_velocity > 7000) {
            this->particle_shape.setFillColor(red);
        }
    }

    float smoothingKernel(float dst, float smoothing_radius) {
        if (dst >= smoothing_radius) return 0;
        
        double volume = 3.14 * pow(smoothing_radius, 4) / 6;
        return (smoothing_radius-dst)*(smoothing_radius-dst)/volume;
    }

    float smoothingKernelDerivative(float dst, float smoothing_radius) {
        if (dst >= smoothing_radius) return 0;
        float f = pow(smoothing_radius, 2) - pow(dst, 2);
        float scale = 12 / (pow(smoothing_radius, 4)*3.14);
        return (dst-smoothing_radius)*scale;
    }
};

class Grid {
public:
    float width = screen_width;
    float height = screen_height;
    std::vector<std::vector<std::vector<particle*>>> grid;
    int rows = screen_height/smoothing_radius;
    int cols = screen_width / smoothing_radius;
    float cell_width;
    float cell_height;

    Grid() {
        this->cell_height = width / cols;
        this->cell_width = height / rows;
        grid.resize(rows, std::vector<std::vector<particle*>>(cols));
    }

    int getRow(float y) const {
        if (y > 0) {
            return std::min(static_cast<int>(std::floor(y / cell_height)), rows-1 );
        }
       
    }

    int getCol(float x) const {
        if (x > 0) {
            return std::min(static_cast<int>(std::floor(x / cell_width)), cols-1 );
        }
       
    }

    void addParticle(particle* particle1){
        int row = this->getRow(particle1->position.y);
        int col = this->getCol(particle1->position.x);
        grid[row][col].push_back(particle1);
    }
    
    std::vector<particle*>getParticles(int row, int col) {
        if (row >= 0 && row <10 && col >= 0 && col <16) {
            return grid[row][col];
        }
    }

    std::vector<int> getNeighbours(int row, int col) {
        if (row >= 0 && row <= 10 && col >= 0 && col <= 16) {
            int start_row = std::max(row - 1, 0);
            int end_row = std::min(row + 1, rows-1);
            int start_col = std::max(col - 1, 0);
            int end_col = std::min(col + 1, cols-1);
            std::vector<int> neighbours = { start_row,end_row,start_col,end_col };
            return neighbours;
        }
    }
};



float densityToPressure(float density) {
    return pressureMultiplier * (density - target_density);
}

static float calculateDensity(std::vector<particle> &particles, sf::Vector2f &poi) {
    float density = 0;
    for (auto& particle1 : particles) {
        float dst = magnitude(particle1.position - poi);
        if (dst < smoothing_radius) {
            density += particle1.smoothingKernel(dst, smoothing_radius) * 10000;
        }
    }
    return density;
   
}

static float calculate_shared_pressure(float density1, float density2) {
    return (densityToPressure(density1) + densityToPressure(density2)) / 2;
}

sf::Vector2f calculatePressureForceOptimized(std::vector<std::vector<particle*>>& neighbours, particle* particle1, std::map<int, float>& densities) {
    sf::Vector2f gradient;
    sf::Vector2f position_lookup;
    position_lookup.x = particle1->position.x + particle1->velocity.x *0.001;
    position_lookup.y = particle1->position.y + particle1->velocity.y * 0.001;
    for (auto& vector1 : neighbours) {
        for (particle* particle2 : vector1) {
            float dst = magnitude(particle2->position - position_lookup);
            if (dst != 0) {
                sf::Vector2f direction = (particle2->position - position_lookup) / dst;
                float slope = particle2->smoothingKernelDerivative(dst, smoothing_radius);
                float density = densities[particle2->id];
                float shared_pressure = calculate_shared_pressure(density, densities[particle1->id]);
                if (density != 0) {
                    gradient += shared_pressure * slope * particle2->mass * direction / density;
                }
            }
        }
    }
    return gradient;

}

static void updateDensities(std::vector<particle> &particles, std::map<int,float>&densities,int start, int end) {
    
    for (int i =start; i <end; ++i) {
        sf::Vector2f position_lookup;
        position_lookup.x = particles[i].position.x + particles[i].velocity.x * 0.001;
        position_lookup.y = particles[i].position.y + particles[i].velocity.y * 0.001;
        densities[particles[i].id] = calculateDensity(particles, position_lookup);
    }
}

sf::Vector2f mouseForce(sf::Vector2i pos, float radius_impact, std::vector<particle*>& particles,std::map<int,sf::Vector2f> &velocities,float strength) {
    sf::Vector2f mouse_force;
    for (auto& particle1 : particles) {
        sf::Vector2f mouse_position;
        
        mouse_position.x = pos.x;
        mouse_position.y = pos.y;
        float dst = magnitude(mouse_position-particle1->position);
        if (dst < radius_impact) {
            sf::Vector2f direction = mouse_position - particle1->position;
            float coefficient = 1 - dst / radius_impact;
            mouse_force += (direction  * strength-velocities[particle1->id]) * coefficient;
        }
    }
    return mouse_force;
}

static void updateGridCols(Grid& grid1, int col, std::map<int, float>& densities, sf::RenderWindow& window, std::map<int,sf::Vector2f> &velocities) {
    for (int i=0; i < grid1.rows; ++i) {
        std::vector<particle*>current_grid_particles = grid1.getParticles(i,col);
        std::vector<int> neighbours = grid1.getNeighbours(i, col);
        int r1, r2, c1, c2;
        r1 = neighbours[0];
        r2 = neighbours[1];
        c1 = neighbours[2];
        c2 = neighbours[3];
        std::vector<std::vector<particle*>> neighbour_particles;
        for (int i = r1; i <= r2; ++i) {
            for (int j = c1; j <= c2; ++j) {
                neighbour_particles.push_back(grid1.getParticles(i, j));
            }
        }
        for (particle *particle1 : current_grid_particles) {
            sf::Vector2f pressure_force = calculatePressureForceOptimized(neighbour_particles, particle1, densities);
            if (magnitude(pressure_force) > pow(10, 8)) {
                pressure_force.x = pressure_force.x / 100; //comment out these lines if gravity off
                pressure_force.y = pressure_force.y / 100;
            }
            if (sf::Mouse::isButtonPressed(sf::Mouse::Left)) {
                sf::Vector2f force1= mouseForce(sf::Mouse::getPosition(window), impact_radius_mouse2, current_grid_particles,std::ref(velocities),strength2);
                particle1->update(pressure_force + force1);
            }
            else if (sf::Mouse::isButtonPressed(sf::Mouse::Right)) {
                sf::Vector2f force1 = mouseForce(sf::Mouse::getPosition(window), impact_radius_mouse, current_grid_particles, std::ref(velocities),-strength);
                particle1->update(pressure_force + force1);
            }
            else {
                particle1->update(pressure_force);
            }
 
        }
    }
}

int main()
{
    sf::RenderWindow window(sf::VideoMode(screen_width, screen_height), "Fluid Simulation");
    std::random_device rd{};
    std::mt19937 gen{ rd() };
    std::normal_distribution<> d_position{ 300, 250 };
    number_threads = std::thread::hardware_concurrency();
    if (!number_threads) {
        number_threads = 4; // a minimum number of threads to spawn
    }
    num_particles = number_threads * 200;

    window.setFramerateLimit(30);
    std::vector <particle> particles;
    sf::Vector2f pressureForce;
    std::map<int, float> densities;
    std::map<int, sf::Vector2f> velocities;
    std::map<int, float> speeds;
   
    int num_horizontal = screen_width / smoothing_radius;
    int num_vertical = screen_height / smoothing_radius;
    
    for (int i = 0; i < num_particles; ++i) {
            particles.push_back(std::move((particle(i, d_position(gen), d_position(gen), 0, 0))));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    while (window.isOpen()) {
        auto start = std::chrono::high_resolution_clock::now();
        std::vector<std::thread> update_density_threads;
        std::vector<std::thread> update_grid_threads;
        float mouse_click_x;
        float mouse_click_y;
        Grid grid1;

        sf::Event event;
        window.clear();
        for (particle&particle1 :particles) {
            particle1.boundary_check();

            grid1.addParticle(&particle1);

            velocities[particle1.id] = particle1.velocity;
            speeds[particle1.id] = magnitude(particle1.velocity);

            particle1.render(window);
        }
     
        window.display();        
        for (int i = 0; i < num_particles; i += num_particles / number_threads) {
            int start = i;
            int end = i + num_particles / number_threads;
            std::thread t1(updateDensities,std::ref(particles), std::ref(densities),start,end);
            update_density_threads.push_back(move(t1));
        }
        for (auto& thread1 : update_density_threads) {
            thread1.join();
        }
        
        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) window.close();
            if (sf::Keyboard::isKeyPressed(sf::Keyboard::Escape)) window.close();
        }
     
        for (int i = 0; i < 16; ++i) {
            std::thread t1(updateGridCols, std::ref(grid1),i, std::ref(densities),std::ref(window),std::ref(velocities));
            update_grid_threads.push_back(move(t1));
        }

        for (auto& thread1 : update_grid_threads) {
            thread1.join();
        }
        //map_average_max(speeds);
       
        auto end = std::chrono::high_resolution_clock::now();
        //std::cout << "The time taken is " << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count()<<"\n";
    }
    return 0;
}

