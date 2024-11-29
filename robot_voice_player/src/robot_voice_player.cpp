#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <SFML/Audio.hpp>
#include <string>
#include <unordered_map>
class RobotVoicePlayer : public rclcpp::Node
{
public:
    RobotVoicePlayer() : Node("robot_voice_player")
    {
        RCLCPP_INFO(this->get_logger(), "Robot voice player node started.");
        // 设置音频文件的映射
        sound_files_["到达战车营"] = "/home/bei/dev_ws/src/robot_voice_player/audio/arrive_tank_camp.wav";
        sound_files_["到达步兵营"] = "/home/bei/dev_ws/src/robot_voice_player/audio/arrive_infantry_camp.wav";
        sound_files_["到达骑兵营"] = "/home/bei/dev_ws/src/robot_voice_player/audio/arrive_cavalry_camp.wav";
        sound_files_["到达粮仓"]   = "/home/bei/dev_ws/src/robot_voice_player/audio/arrive_granary_camp.wav";

        // 订阅/robotvoice话题
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/robotvoice", 10,
            std::bind(&RobotVoicePlayer::voiceCallback, this, std::placeholders::_1));
    }

private:
    void voiceCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        std::string command = msg->data;
        std::cout << "commmand is: " << command << std::endl;

        // 检查是否有对应的音频文件
        if (sound_files_.find(command) != sound_files_.end())
        {
            std::string audio_file = sound_files_[command];

            // 播放对应的音频
            playAudio(audio_file);
        }
        else
        {
            RCLCPP_WARN(this->get_logger(), "Unknown command: %s", command.c_str());
        }
    }

    void playAudio(const std::string &audio_file)
    {
        sf::SoundBuffer buffer;
        if (buffer.loadFromFile(audio_file))
        {
            sf::Sound sound;
            sound.setBuffer(buffer);
            sound.play();

            // 等待音频播放完成
            while (sound.getStatus() == sf::Sound::Playing)
            {
                rclcpp::sleep_for(std::chrono::milliseconds(100));
            }
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load audio file: %s", audio_file.c_str());
        }
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    std::unordered_map<std::string, std::string> sound_files_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RobotVoicePlayer>());
    rclcpp::shutdown();
    return 0;
}
