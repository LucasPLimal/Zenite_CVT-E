// Detector de marcadores ArUco do Projeto Zênite.
//
// Escrito para o OpenCV 4.6 (API antiga: opencv2/aruco.hpp).
//
// Para cada marcador encontrado na imagem BGR, devolve:
//   • id     -> qual robô é;
//   • center -> centro do marcador (posição do robô);
//   • front  -> ponto médio da aresta SUPERIOR do marcador impresso
//               (frente do robô);
//   • corners-> os 4 cantos (para desenhar).
//
// Montagem no robô: o centro do marcador sobre o centro do robô e a
// aresta superior do marcador voltada para a frente.
//
// Local sugerido: include/localization_node/aruco_tracker.hpp
#pragma once

#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>

#include <vector>

class ArucoTracker {

public:

    // Resultado para um marcador (tudo em pixels).
    struct Detection {

        // ID do marcador (um por robô).
        int id;

        // Centro do marcador.
        cv::Point2f center;

        // Frente do robô: meio da aresta superior do marcador.
        cv::Point2f front;

        // Cantos na ordem: sup-esq, sup-dir, inf-dir, inf-esq
        // (relativos ao marcador impresso, então giram junto com o robô).
        std::vector<cv::Point2f> corners;
    };

    ArucoTracker()
        : dict_(cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50)),
          params_(cv::aruco::DetectorParameters::create())
    {
        // Refina os cantos com precisão sub-pixel (melhora posição e ângulo).
        params_->cornerRefinementMethod = cv::aruco::CORNER_REFINE_SUBPIX;
    }

    // Procura todos os marcadores na imagem.
    // Devolve um vetor vazio se nenhum for encontrado.
    std::vector<Detection> detect(const cv::Mat& bgr) const
    {
        std::vector<Detection> out;
        if (bgr.empty()) return out;

        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(bgr, dict_, corners, ids, params_);

        for (size_t i = 0; i < ids.size(); ++i) {
            const auto& c = corners[i];

            Detection d;
            d.id      = ids[i];
            d.center  = (c[0] + c[1] + c[2] + c[3]) * 0.25f;
            d.front   = (c[0] + c[1]) * 0.5f;
            d.corners = c;
            out.push_back(d);
        }

        return out;
    }

private:

    cv::Ptr<cv::aruco::Dictionary> dict_;
    cv::Ptr<cv::aruco::DetectorParameters> params_;
};
