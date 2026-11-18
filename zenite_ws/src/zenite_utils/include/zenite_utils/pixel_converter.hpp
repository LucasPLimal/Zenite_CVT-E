#ifndef ZENITE_UTILS_PIXEL_CONVERTER_HPP
#define ZENITE_UTILS_PIXEL_CONVERTER_HPP

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include <fstream>

namespace zenite_utils
{

class PixelConverter
{
public:
    PixelConverter() = default;

    // Define os 4 pontos de referência (em pixels) e suas posições reais (em metros)
    void setReferencePoints(const std::vector<cv::Point2f>& pixel_points,
                            const std::vector<cv::Point2f>& real_points);

    // Converte pixel -> metro usando homografia
    cv::Point2f pixelToMeter(const cv::Point2f& pixel_point) const;

    // Converte metro -> pixel usando homografia inversa
    cv::Point2f meterToPixel(const cv::Point2f& real_point) const;

    // Salva a homografia e pontos em YAML
    void saveToYaml(const std::string& path) const;

    // Carrega a homografia e pontos de YAML
    void loadFromYaml(const std::string& path);

    // Alteração feita por Danilo: indica se a homografia já foi carregada ou
    // calculada. Sem ela, pixelToMeter() lança exceção do OpenCV.
    bool ready() const { return !H_.empty(); }

private:
    std::vector<cv::Point2f> pixel_points_;
    std::vector<cv::Point2f> real_points_;
    cv::Mat H_;        // Homografia (imagem -> mundo)
    cv::Mat H_inv_;    // Homografia inversa (mundo -> imagem)
};

}  // namespace zenite_utils

namespace YAML
{
template<>
struct convert<cv::Point2f>
{
    static Node encode(const cv::Point2f& rhs)
    {
        Node node;
        node.push_back(rhs.x);
        node.push_back(rhs.y);
        return node;
    }

    static bool decode(const Node& node, cv::Point2f& rhs)
    {
        if (!node.IsSequence() || node.size() != 2)
            return false;
        rhs.x = node[0].as<float>();
        rhs.y = node[1].as<float>();
        return true;
    }
};
}  // namespace YAML

#endif // ZENITE_UTILS_PIXEL_CONVERTER_HPP
