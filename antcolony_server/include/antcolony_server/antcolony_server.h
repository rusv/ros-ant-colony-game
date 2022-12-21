// Copyright 2022 Vitalii Russinkovskii

#ifndef ANTCOLONY_SERVER_ANTCOLONY_SERVER_H
#define ANTCOLONY_SERVER_ANTCOLONY_SERVER_H

#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include <opencv2/highgui/highgui.hpp>

#include <antcolony_msgs/Action.h>
#include <antcolony_msgs/Ant.h>
#include <antcolony_msgs/Hive.h>
#include <antcolony_msgs/Order.h>
#include <antcolony_msgs/Player.h>
#include <antcolony_msgs/State.h>
#include <antcolony_msgs/Sugar.h>

#include <antcolony_msgs/Play.h>
#include <antcolony_msgs/Register.h>

#define MAP_SIZE 40

#define END_TICK 500

#define ANT_ATTACK_HIT 1
#define SUGAR_ATTACK_HIT 1
#define HIVE_ATTACK_HIT 1

#define INITIAL_HIVE_HEALTH 20
#define INITIAL_SUGAR_AMOUNT 10

#define INITIAL_ANT_HEALTH 5
#define BABY_ANT_SUGAR INITIAL_ANT_HEALTH

#define SUGAR_TO_HIVE_MIN_MANHATTAN 5
#define INITIAL_SUGAR_PER_PLAYER 40

#define GRAPHIC_CELL_SIZE 20
#define GRAPHIC_MAP_BACKGROUND_COLOR CV_RGB(0xF2, 0xF2, 0xF2)
#define GRAPHIC_MAP_FOREGROUND_COLOR CV_RGB(0x0, 0x0, 0x0)
#define GRAPHIC_MAP_INFO_TEXT_COLOR CV_RGB(0x0, 0x0, 0x0)
#define GRAPHIC_MAP_INFO_ZONE_HEIGHT 40
#define GRAPHIC_MAP_INFO_ZONE_TEXT_OFFSET_X 10
#define GRAPHIC_MAP_INFO_ZONE_TEXT_OFFSET_Y 30

#define GAME_TICK_FREQUENCY 4

class GameEngine {
public:
  // Constructs game engine
  GameEngine();

  // Gets current state of the game
  void getCurrentState(antcolony_msgs::State &state);

  // Sets initial state of the game
  void setInitialState(antcolony_msgs::State &state, int player_one_id,
                       std::string token_one, int player_two_id,
                       std::string token_two);

  void playGameTick(std::vector<antcolony_msgs::Order> &orders);

private:
  // Last assigned unique identifier
  int last_id = 0;
  int game_tick = 0;
  int max_coordinate;

  // Security tokens of players
  std::unordered_map<std::string, int> active_tokens;

  // Alive ants
  std::unordered_map<int, antcolony_msgs::Ant> ants;
  // Alive hives
  std::unordered_map<int, antcolony_msgs::Hive> hives;
  // Game players
  std::unordered_map<int, antcolony_msgs::Player> players;
  // Available sugars
  std::unordered_map<int, antcolony_msgs::Sugar> sugars;

  // Baby ants that grow
  std::vector<antcolony_msgs::Ant> baby_ants;

  // Map occupancy
  std::unordered_map<int, int> occupancy_map;

  // Next unique identifier
  int nextId();

  // Rewinds unique identifier behind used id
  void rewindId(int used_id);

  // Convert coordinates to hash
  int coordinateHash(int x, int y) const;

  // Check if cell of map is occupied
  bool isOccupied(int x, int y) const;

  // Generates neighbors for coordinates
  void generateNeighbors(int x, int y,
                         std::vector<std::pair<int, int>> &neighbors) const;

  // Find next move on the way from start to end
  bool searchPath(int start_x, int start_y, int end_x, int end_y, int &next_x,
                  int &next_y) const;

  // If hive has enough sugar start grow baby ant
  void conceiveBabyAnts();

  // Increase health of baby ants on each move
  void growBabyAnts();

  void nearestUnoccupied(int start_x, int start_y, int &free_x,
                         int &free_y) const;

  // Spawn grown ant on map
  void spawnAnt(const int player_id);

  // Manhattan distance
  int manhattan_distance(int x1, int y1, int x2, int y2) const;

  // Attacks at object at given coordinates
  void attackTarget(int ant_id, int target_x, int target_y);

  void attackHive(int target_id);

  void attackAnt(int target_id);

  void attackSugar(int player_id, int target_id);

  void moveAnt(int ant_id, int target_x, int target_y);

  void processPlayerOrders(std::vector<antcolony_msgs::Order> &orders);
};

class GameServer {
public:
  // Initializes game server
  void init();

  // Plays one game
  void playGame();

private:
  // Tokens of registered clients
  std::unordered_set<std::string> registered_clients;

  // Mutex to synchronize access to orders
  std::mutex orchestration_mutex;

  // Latest orders received for current game tick
  std::unordered_map<std::string, antcolony_msgs::Order> tick_orders;

  std::string player_one_token;
  std::string player_two_token;

  int player_one_id;
  int player_two_id;

  antcolony_msgs::State state;

  GameEngine gameEngine;

  ros::NodeHandle nh;
  std::shared_ptr<image_transport::ImageTransport> image_transport;

  // Publisher of game board image
  image_transport::Publisher game_board_image_pub;

  // Publisher of game state
  ros::Publisher game_state_pub;

  ros::ServiceServer register_service;
  ros::ServiceServer play_service;

  // Game graphics is here
  cv::Mat blue_ant_miniature;
  cv::Mat red_ant_miniature;
  cv::Mat blue_hive_miniature;
  cv::Mat red_hive_miniature;
  cv::Mat white_sugar_miniature;

  // Callback to handle requests to register for a new game match
  bool register_call(antcolony_msgs::RegisterRequest &request,
                     antcolony_msgs::RegisterResponse &response);

  // Callback to submit an order for the current game tick with exclusive access
  bool play_call(antcolony_msgs::PlayRequest &request,
                 antcolony_msgs::PlayResponse &response);

  // Copies orders with exclusive access
  void copyTickOrders(std::vector<antcolony_msgs::Order> &orders);

  // Prints out final game scores
  void reportGameScores(antcolony_msgs::State &state);

  // Generates and publishes image of game map
  void publishGameMapImage();

  // Sets game of two players
  void setupGame();
};

#endif // ANTCOLONY_SERVER_ANTCOLONY_SERVER_H
