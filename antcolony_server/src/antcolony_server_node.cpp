// Copyright 2022 Vitalii Russinkovskii

#include <algorithm>
#include <cstdlib>
#include <deque>
#include <exception>
#include <iostream>
#include <random>
#include <sstream>
#include <thread>
#include <unordered_set>
#include <utility>
#include <vector>


#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/imgproc.hpp>
#include <cv_bridge/cv_bridge.h>

#include "antcolony_server/antcolony_server.h"

GameEngine::GameEngine() {
    max_coordinate = MAP_SIZE - 1;
}

void GameServer::init() {
    // Clear tokens before registering services
    registered_clients.clear();

    // Secret tokens
    const char* token_one_p = std::getenv("ANTCOLONY_PLAYER_ONE_TOKEN");
    const char* token_two_p = std::getenv("ANTCOLONY_PLAYER_TWO_TOKEN");
    player_one_token = std::string(token_one_p);
    player_two_token = std::string(token_two_p);

    // Prepare game to play
    setupGame();

    // Private ROS node
    nh = ros::NodeHandle("~");

    // Load paths for miniatures
    std::cout << "Loadning parameters..." << std::endl;

    std::string blue_ant_image_path, red_ant_image_path, blue_hive_image_path, red_hive_image_path, white_sugar_image_path;
    nh.getParam("blue_ant_image_path", blue_ant_image_path);
    nh.getParam("red_ant_image_path", red_ant_image_path);
    nh.getParam("blue_hive_image_path", blue_hive_image_path);
    nh.getParam("red_hive_image_path", red_hive_image_path);
    nh.getParam("white_sugar_image_path", white_sugar_image_path);

    // Load graphical resources
    blue_ant_miniature = cv::imread(blue_ant_image_path, CV_LOAD_IMAGE_COLOR);
    red_ant_miniature = cv::imread(red_ant_image_path, CV_LOAD_IMAGE_COLOR);
    blue_hive_miniature = cv::imread(blue_hive_image_path, CV_LOAD_IMAGE_COLOR);
    red_hive_miniature = cv::imread(red_hive_image_path, CV_LOAD_IMAGE_COLOR);
    white_sugar_miniature = cv::imread(white_sugar_image_path, CV_LOAD_IMAGE_COLOR);

    // Topic to publish game images
    image_transport = std::make_shared<image_transport::ImageTransport>(nh);
    game_board_image_pub = image_transport->advertise("game_board_image/image", 1);

    // Topic to publish game state
    game_state_pub = nh.advertise<antcolony_msgs::State>("game_state", 1);

    // Register services
    register_service = nh.advertiseService("register_cmd", &GameServer::register_call, this);
    play_service = nh.advertiseService("play_cmd", &GameServer::play_call, this);

}

void drawMapBoard(cv::Mat &board, const int player_one_score, const int player_two_score, const int game_tick) {
    board = cv::Mat(MAP_SIZE * GRAPHIC_CELL_SIZE + GRAPHIC_MAP_INFO_ZONE_HEIGHT, MAP_SIZE * GRAPHIC_CELL_SIZE, CV_8UC3, GRAPHIC_MAP_BACKGROUND_COLOR);

    // Footer with scores and tick
    std::stringstream ss;
    ss << "Red score: " << std::setw(3) << player_one_score << " Blue score: " << std::setw(3) << player_two_score << " tick: " << std::setw(5) << game_tick;

    cv::line(board, cv::Point(0, MAP_SIZE * GRAPHIC_CELL_SIZE), cv::Point(MAP_SIZE * GRAPHIC_CELL_SIZE - 1, MAP_SIZE * GRAPHIC_CELL_SIZE), GRAPHIC_MAP_FOREGROUND_COLOR, 1);
    cv::putText(board,
                ss.str(),
                cv::Point(GRAPHIC_MAP_INFO_ZONE_TEXT_OFFSET_X, MAP_SIZE * GRAPHIC_CELL_SIZE + GRAPHIC_MAP_INFO_ZONE_TEXT_OFFSET_Y),
                cv::FONT_HERSHEY_TRIPLEX,
                1.0,
                GRAPHIC_MAP_INFO_TEXT_COLOR, //font color
                2);
}

void placeMiniature(cv::Mat &board, cv::Mat &miniature, int x, int y) {
    cv::Mat board_roi = board(cv::Rect(x * GRAPHIC_CELL_SIZE, y * GRAPHIC_CELL_SIZE, GRAPHIC_CELL_SIZE, GRAPHIC_CELL_SIZE));
    miniature.copyTo(board_roi);
}

void GameServer::publishGameMapImage() {
    cv::Mat board;
    // Drawing game board
    int player_one_score, player_two_score;
    for(auto &player : state.players) {
        if(player.id == player_one_id) {
            player_one_score = player.score;
        }
        if(player.id == player_two_id) {
            player_two_score = player.score;
        }
    }
    drawMapBoard(board, player_one_score, player_two_score, state.tick);

    // Drawing hives
    for(auto &hive : state.hives) {
        if(hive.player_id == player_one_id) {
            placeMiniature(board, red_hive_miniature, hive.x, hive.y);
        }
        if(hive.player_id == player_two_id) {
            placeMiniature(board, blue_hive_miniature, hive.x, hive.y);
        }
    }

    // Drawing ants
    for(auto &ant : state.ants) {
        if(ant.player_id == player_one_id) {
            placeMiniature(board, red_ant_miniature, ant.x, ant.y);
        }
        if(ant.player_id == player_two_id) {
            placeMiniature(board, blue_ant_miniature, ant.x, ant.y);
        }
    }

    // Drawing sugar
    for(auto &sugar : state.sugars) {
        placeMiniature(board, white_sugar_miniature, sugar.x, sugar.y);
    }

    // Publishing image
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", board).toImageMsg();
    game_board_image_pub.publish(msg);

}

int GameEngine::coordinateHash(const int x, const int y) const {
    return MAP_SIZE * y + x;
}

bool GameEngine::isOccupied(const int x, const int y) const {
    int key = coordinateHash(x, y);
    auto search = occupancy_map.find(key);
    return search != occupancy_map.end();
}

void GameEngine::generateNeighbors(const int x, const int y, std::vector<std::pair<int, int>> &neighbors) const {
    neighbors.clear();
    if(x > 0) {
        neighbors.emplace_back(x - 1, y);
    }
    if(x < max_coordinate) {
        neighbors.emplace_back(x + 1, y);
    }
    if(y > 0) {
        neighbors.emplace_back(x, y - 1);
    }
    if(y < max_coordinate) {
        neighbors.emplace_back(x, y + 1);
    }
}

bool GameEngine::searchPath(const int start_x, const int start_y, const int end_x, const int end_y, int &next_x, int &next_y) const {
    std::unordered_set<int> visited;
    std::unordered_map<int, std::pair<int, int>> ancestors;
    std::deque<std::pair<int, int>> search_deque;
    std::vector<std::pair<int, int>> neighbors(4);

    search_deque.emplace_back(start_x, start_y);
    ancestors.emplace(coordinateHash(start_x, start_y), std::make_pair(start_x, start_y));
    visited.emplace(coordinateHash(start_x, start_y));
    while (!search_deque.empty()) {
        if (search_deque.front().first == end_x && search_deque.front().second == end_y) {
            std::pair<int, int> current = search_deque.front();
            while (true) {
                std::pair<int, int> ancestor = ancestors.at(coordinateHash(current.first, current.second));
                if (ancestor.first == start_x && ancestor.second == start_y) {
                    next_x = current.first;
                    next_y = current.second;
                    return true;
                }
                current = ancestor;
            }
        }
        generateNeighbors(search_deque.front().first, search_deque.front().second, neighbors);
        for(const auto &neighbor : neighbors) {
            int key = coordinateHash(neighbor.first, neighbor.second);
            // Already visited
            if(auto search = visited.find(key); search != visited.end()) {
                continue;
            }
            // Occupied by something
            if(auto search = occupancy_map.find(key); search != occupancy_map.end()) {
                continue;
            }
            // Adding search candidate
            search_deque.emplace_back(neighbor.first, neighbor.second);
            ancestors.emplace(key, search_deque.front());
            visited.emplace(coordinateHash(neighbor.first, neighbor.second));
        }
        search_deque.pop_front();
    }
    // Path not found
    return false;
}

int GameEngine::manhattan_distance(const int x1, const int y1, const int x2, const int y2) const {
    return std::abs(x1 - x2) + std::abs(y1 - y2);
}

void GameEngine::attackTarget(const int ant_id, const int target_x, const int target_y) {
    // Ant should exist
    auto search_ant = ants.find(ant_id);
    if(search_ant == ants.end()) {
        return;
    }
    // Ant should be in adjacent cell
    if(manhattan_distance(search_ant->second.x, search_ant->second.y, target_x, target_y) > 1) {
        return;
    }
    auto search_target = occupancy_map.find(coordinateHash(target_x, target_y));
    if(search_target == occupancy_map.end()) {
        return;
    }
    int player_id = search_ant->second.player_id;

    attackAnt(search_target->second);
    attackHive(search_target->second);
    attackSugar(player_id, search_target->second);
}

void GameEngine::attackAnt(const int target_id) {
    auto search = ants.find(target_id);
    if(search == ants.end()) {
        return;
    }
    search->second.health -= ANT_ATTACK_HIT;
    // Check if ant is killed
    if(search->second.health <= 0) {
        occupancy_map.erase(coordinateHash(search->second.x, search->second.y));
        ants.erase(search);
    }
}

void GameEngine::attackSugar(const int player_id, const int target_id) {
    auto search = sugars.find(target_id);
    if(search == sugars.end()) {
        return;
    }
    int sugar_produced = std::min(search->second.amount, SUGAR_ATTACK_HIT);
    players.find(player_id)->second.score += sugar_produced;
    players.find(player_id)->second.sugar += sugar_produced;

    search->second.amount -= SUGAR_ATTACK_HIT;
    if(search->second.amount <= 0) {
        occupancy_map.erase(coordinateHash(search->second.x, search->second.y));
        sugars.erase(search);
    }
}

void GameEngine::attackHive(const int target_id) {
    auto search = hives.find(target_id);
    if(search == hives.end()) {
        return;
    }
    search->second.health -= HIVE_ATTACK_HIT;
    if(search->second.health <= 0) {
        occupancy_map.erase(coordinateHash(search->second.x, search->second.y));
        const int target_player_id = search->second.player_id;
        hives.erase(search);
        baby_ants.erase(std::remove_if(baby_ants.begin(),
                                  baby_ants.end(),
                                  [target_player_id](antcolony_msgs::Ant x){return x.player_id == target_player_id;}),
                   baby_ants.end());
    }
}

int GameEngine::nextId() {
    return ++last_id;
}

void GameEngine::rewindId(int used_id) {
    if(last_id < used_id) {
        last_id = used_id;
    }
}

void GameEngine::conceiveBabyAnts() {
    for(auto &player : players) {
        while (player.second.sugar >= BABY_ANT_SUGAR) {
            antcolony_msgs::Ant baby_ant;
            baby_ant.player_id = player.second.id;
            baby_ant.health = 0;
            baby_ants.push_back(baby_ant);
            player.second.sugar -= BABY_ANT_SUGAR;
        }
    }
}

void GameEngine::nearestUnoccupied(const int start_x, const int start_y, int &free_x, int &free_y) const {
    std::unordered_set<int> visited;
    std::deque<std::pair<int, int>> search_deque;
    std::vector<std::pair<int, int>> neighbors(4);

    search_deque.emplace_back(start_x, start_y);
    visited.emplace(coordinateHash(start_x, start_y));

    while (!search_deque.empty()) {
        if(!isOccupied(search_deque.front().first, search_deque.front().second)) {
            free_x = search_deque.front().first;
            free_y = search_deque.front().second;
            return;
        }
        generateNeighbors(search_deque.front().first, search_deque.front().second, neighbors);
        for(const auto &neighbor : neighbors) {
            int key = coordinateHash(neighbor.first, neighbor.second);
            // Already visited
            if(auto search = visited.find(key); search != visited.end()) {
                continue;
            }
            // Adding search candidate
            search_deque.emplace_back(neighbor.first, neighbor.second);
            visited.emplace(coordinateHash(neighbor.first, neighbor.second));
        }
        search_deque.pop_front();
    }
    throw std::runtime_error("No free placement was found");
}

void GameEngine::spawnAnt(const int player_id) {
    for(auto &hive : hives) {
        if(hive.second.player_id == player_id) {
            int free_x, free_y;
            nearestUnoccupied(hive.second.x, hive.second.y, free_x, free_y);
            antcolony_msgs::Ant ant;
            ant.id = nextId();
            ant.player_id = player_id;
            ant.health = INITIAL_ANT_HEALTH;
            ant.x = free_x;
            ant.y = free_y;
            occupancy_map[coordinateHash(free_x, free_y)] = ant.id;
            ants[ant.id] = ant;
        }
    }
}

void GameEngine::growBabyAnts() {
    for (auto &baby_ant: baby_ants) {
        baby_ant.health++;
        if(baby_ant.health >= INITIAL_ANT_HEALTH) {
            spawnAnt(baby_ant.player_id);
        }
    }
    baby_ants.erase(std::remove_if(baby_ants.begin(),
                                   baby_ants.end(),
                                   [](antcolony_msgs::Ant x){return x.health >= INITIAL_ANT_HEALTH;}),
                    baby_ants.end());
}

void GameEngine::moveAnt(const int ant_id, const int target_x, const int target_y) {
    auto search_ant = ants.find(ant_id);
    if(search_ant == ants.end()) {
        return;
    }
    int next_x, next_y;
    if(searchPath(search_ant->second.x, search_ant->second.y, target_x, target_y, next_x, next_y)) {
        occupancy_map.erase(coordinateHash(search_ant->second.x, search_ant->second.y));
        occupancy_map[coordinateHash(next_x, next_y)] = ant_id;
        search_ant->second.x = next_x;
        search_ant->second.y = next_y;
    }
}

void GameEngine::processPlayerOrders(std::vector<antcolony_msgs::Order> &orders) {
    std::vector<antcolony_msgs::Action> accepted_actions;
    std::unordered_set<int> assigned_ant;
    // Security and integrity verification
    for(auto &order : orders) {
        auto search_token = active_tokens.find(order.token);
        if(search_token == active_tokens.end()) {
            continue;
        }
        const int player_id = search_token->second;
        for(auto &action : order.actions) {
            auto ant_search = ants.find(action.id);
            if(ant_search == ants.end()) {
                continue;
            }
            if(ant_search->second.player_id != player_id) {
                continue;
            }
            if(action.x < 0 || action.x > max_coordinate || action.y < 0 || action.y > max_coordinate) {
                continue;
            }
            auto search_assigned_ant = assigned_ant.find(action.id);
            if(search_assigned_ant != assigned_ant.end()) {
                continue;
            }
            accepted_actions.push_back(action);
            assigned_ant.emplace(action.id);
        }
    }

    auto rd = std::random_device {};
    auto rng = std::default_random_engine { rd() };
    std::shuffle(std::begin(accepted_actions), std::end(accepted_actions), rng);

    for(const auto &action : accepted_actions) {
        switch (action.action_type) {
            case antcolony_msgs::Action::MOVE_ACTION:
                moveAnt(action.id, action.x, action.y);
                break;
            case antcolony_msgs::Action::ATTACK_ACTION:
                attackTarget(action.id, action.x, action.y);
                break;
        }
    }
}

void GameEngine::playGameTick(std::vector<antcolony_msgs::Order> &orders) {
    game_tick++;
    processPlayerOrders(orders);
    growBabyAnts();
    conceiveBabyAnts();
}

void GameServer::setupGame() {
    int max_coordinate = MAP_SIZE - 1;
    player_one_id = 1;
    player_two_id = 2;
    int nextId = 2;

    // Players
    state.players.clear();
    antcolony_msgs::Player player_one;
    antcolony_msgs::Player player_two;
    player_one.id = player_one_id;
    player_two.id = player_two_id;
    player_one.score = 0;
    player_two.score = 0;
    player_one.sugar = 0;
    player_two.sugar = 0;
    state.players.push_back(player_one);
    state.players.push_back(player_two);

    // Players hives
    state.hives.clear();
    antcolony_msgs::Hive hive_player_one;
    antcolony_msgs::Hive hive_player_two;
    hive_player_one.id = ++nextId;
    hive_player_two.id = ++nextId;
    hive_player_one.player_id = player_one_id;
    hive_player_two.player_id = player_two_id;
    hive_player_one.health = INITIAL_HIVE_HEALTH;
    hive_player_two.health = INITIAL_HIVE_HEALTH;
    hive_player_one.x = 0;
    hive_player_one.y = 0;
    hive_player_two.x = max_coordinate;
    hive_player_two.y = max_coordinate;
    state.hives.push_back(hive_player_one);
    state.hives.push_back(hive_player_two);

    // Players ants
    state.ants.clear();
    antcolony_msgs::Ant ant_player_one;
    antcolony_msgs::Ant ant_player_two;
    ant_player_one.id = ++nextId;
    ant_player_two.id = ++nextId;;
    ant_player_one.player_id = player_one_id;
    ant_player_two.player_id = player_two_id;
    ant_player_one.health = INITIAL_ANT_HEALTH;
    ant_player_two.health = INITIAL_ANT_HEALTH;
    ant_player_one.x = 1;
    ant_player_one.y = 1;
    ant_player_two.x = max_coordinate - 1;
    ant_player_two.y = max_coordinate - 1;
    state.ants.push_back(ant_player_one);
    state.ants.push_back(ant_player_two);

    // Sugar
    state.sugars.clear();
    std::vector<std::pair<int, int>> sugar_location_candidates;
    for(int i = 0; i < MAP_SIZE; i++) {
        for(int j = 0; j < MAP_SIZE; j++) {
            if ((i + j) >= SUGAR_TO_HIVE_MIN_MANHATTAN && (i + j) < max_coordinate) {
                sugar_location_candidates.emplace_back(i, j);
            }
        }
    }
    auto rd = std::random_device {};
    auto rng = std::default_random_engine { rd() };
    std::shuffle(std::begin(sugar_location_candidates), std::end(sugar_location_candidates), rng);
    for(int i = 0; i < INITIAL_SUGAR_PER_PLAYER; i++) {
        antcolony_msgs::Sugar sugar_one;
        antcolony_msgs::Sugar sugar_two;
        sugar_one.id = ++nextId;
        sugar_two.id = ++nextId;
        sugar_one.amount = INITIAL_SUGAR_AMOUNT;
        sugar_two.amount = INITIAL_SUGAR_AMOUNT;
        sugar_one.x = sugar_location_candidates[i].first;
        sugar_one.y = sugar_location_candidates[i].second;
        sugar_two.x = max_coordinate - sugar_location_candidates[i].first;
        sugar_two.y = max_coordinate - sugar_location_candidates[i].second;
        state.sugars.push_back(sugar_one);
        state.sugars.push_back(sugar_two);
    }

    gameEngine.setInitialState(state, player_one_id, player_one_token, player_two_id, player_two_token);
}

void GameEngine::getCurrentState(antcolony_msgs::State &state) {
    // Game tick
    state.tick = game_tick;

    // Players
    state.players.clear();
    for(auto &player : players) {
        state.players.push_back(player.second);
    }

    // Ants
    state.ants.clear();
    for(auto &ant : ants) {
        state.ants.push_back(ant.second);
    }

    // Hives
    state.hives.clear();
    for(auto &hive : hives) {
        state.hives.push_back(hive.second);
    }

    // Sugars
    state.sugars.clear();
    for(auto &sugar : sugars) {
        state.sugars.push_back(sugar.second);
    }
}

void GameEngine::setInitialState(antcolony_msgs::State &state, const int player_one_id, const std::string token_one,
                                 const int player_two_id, const std::string token_two) {
    // Reset game tick
    game_tick = 0;

    // Reset last identifier
    last_id = 0;

    // Occupancy map
    occupancy_map.clear();

    // Players tokens
    active_tokens.clear();
    active_tokens[token_one] = player_one_id;
    active_tokens[token_two] = player_two_id;

    players.clear();
    for(auto &player : state.players) {
        players[player.id] = player;
        rewindId(player.id);
    }

    // Players hives
    hives.clear();
    for(auto &hive : state.hives) {
        hives[hive.id] = hive;
        occupancy_map[coordinateHash(hive.x, hive.y)] = hive.id;
        rewindId(hive.id);
    }

    // Baby ants
    baby_ants.clear();

    // Players ants
    ants.clear();
    for(auto &ant : state.ants) {
        ants[ant.id] = ant;
        occupancy_map[coordinateHash(ant.x, ant.y)] = ant.id;
        rewindId(ant.id);
    }

    // Sugar
    sugars.clear();
    for(auto &sugar : state.sugars) {
        sugars[sugar.id] = sugar;
        occupancy_map[coordinateHash(sugar.x, sugar.y)] = sugar.id;
        rewindId(sugar.id);
    }
}

void GameServer::copyTickOrders(std::vector<antcolony_msgs::Order> &orders) {
    std::lock_guard<std::mutex> lock(orchestration_mutex);
    orders.clear();
    for(auto &tick_order : tick_orders) {
        orders.push_back(tick_order.second);
    }
}

void GameServer::reportGameScores(antcolony_msgs::State &state) {
    int player_one_score;
    int player_two_score;
    for(auto &player : state.players) {
        if(player.id == player_one_id) {
            player_one_score = player.score;
        }
        if(player.id == player_two_id) {
            player_two_score = player.score;
        }
    }
    std::cout << "Player one score: " << player_one_score << " Player two score: " << player_two_score << std::endl;
}

void GameServer::playGame() {
    ros::Rate game_tick_rate(GAME_TICK_FREQUENCY);
    std::vector<antcolony_msgs::Order> orders;
    while (nh.ok()) {
        if(registered_clients.size() < 2) {
            continue;
        }
        gameEngine.getCurrentState(state);
        if(state.tick >= END_TICK) {
            reportGameScores(state);
            ros::shutdown();
            break;
        }
        game_state_pub.publish(state);
        publishGameMapImage();
        game_tick_rate.sleep();
        copyTickOrders(orders);
        gameEngine.playGameTick(orders);
    }
}

bool GameServer::register_call(antcolony_msgs::RegisterRequest &request, antcolony_msgs::RegisterResponse &response) {
    std::lock_guard<std::mutex> lock(orchestration_mutex);
    if(request.token == player_one_token || request.token == player_two_token) {
        registered_clients.emplace(request.token);
        response.status = antcolony_msgs::RegisterResponse::STATUS_OK;
        response.error_message = "";
    } else {
        response.status = antcolony_msgs::RegisterResponse::STATUS_ERROR;
        response.error_message = "Client presented invalid token: " + request.token;
    }
    return true;
}

bool GameServer::play_call(antcolony_msgs::PlayRequest &request, antcolony_msgs::PlayResponse &response) {
    std::lock_guard<std::mutex> lock(orchestration_mutex);
    if(request.order.token == player_one_token || request.order.token == player_two_token) {
        tick_orders[request.order.token] = request.order;
        response.status = antcolony_msgs::PlayResponse::STATUS_OK;
        response.error_message = "";
    } else {
        response.status = antcolony_msgs::PlayResponse::STATUS_ERROR;
        response.error_message = "Client presented invalid token: " + request.order.token;
    }
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "antcolony_server_node");
    GameServer gameServer;
    std::cout << "Game server initialization..." << std::endl;
    gameServer.init();
    std::thread t(&GameServer::playGame, &gameServer);
    std::cout << "Game server is started." << std::endl;
    ros::spin();
}