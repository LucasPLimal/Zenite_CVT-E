// Inclui o arquivo de cabeçalho da classe PixelConverter.
#include "zenite_utils/pixel_converter.hpp"

// Biblioteca utilizada para impressão de mensagens no terminal.
#include <iostream>

namespace zenite_utils
{
// Início do namespace do projeto Zênite.

// ============================================================================
// Define os pontos de referência e calcula a homografia
// ============================================================================

void PixelConverter::setReferencePoints(const std::vector<cv::Point2f>& pixel_points,
                                        const std::vector<cv::Point2f>& real_points)
{
    // Verifica se foram fornecidos exatamente quatro pontos para cada conjunto.
    if (pixel_points.size() != 4 || real_points.size() != 4)
    {
        // Caso contrário, informa o erro.
        std::cerr << "[PixelConverter] Precisam ser 4 pontos de referência." << std::endl;

        // Encerra a função sem calcular a homografia.
        return;
    }

    // Armazena os pontos em pixels recebidos.
    pixel_points_ = pixel_points;

    // Armazena os pontos reais correspondentes.
    real_points_ = real_points;

    // Calcula a matriz de homografia que transforma
    // coordenadas da imagem (pixels) em coordenadas do mundo (metros).
    H_ = cv::findHomography(pixel_points_, real_points_);

    // Calcula automaticamente a matriz inversa,
    // utilizada para converter metros em pixels.
    H_inv_ = H_.inv();

    // Exibe mensagem indicando que a homografia foi calculada.
    std::cout << "[PixelConverter] Homografia calculada." << std::endl;
}

// ============================================================================
// Conversão Pixel → Metro
// ============================================================================

cv::Point2f PixelConverter::pixelToMeter(const cv::Point2f& pixel_point) const
{
    // Cria um vetor contendo o ponto recebido.
    std::vector<cv::Point2f> src = {pixel_point}, dst;

    // Aplica a transformação projetiva utilizando a matriz H.
    // O resultado será armazenado em dst.
    cv::perspectiveTransform(src, dst, H_);

    // Retorna o primeiro (e único) ponto convertido.
    return dst[0];
}

// ============================================================================
// Conversão Metro → Pixel
// ============================================================================

cv::Point2f PixelConverter::meterToPixel(const cv::Point2f& real_point) const
{
    // Cria um vetor contendo o ponto em coordenadas reais.
    std::vector<cv::Point2f> src = {real_point}, dst;

    // Aplica a homografia inversa para converter
    // coordenadas reais em coordenadas da imagem.
    cv::perspectiveTransform(src, dst, H_inv_);

    // Retorna o ponto convertido.
    return dst[0];
}

// ============================================================================
// Salva os pontos de calibração em um arquivo YAML
// ============================================================================

void PixelConverter::saveToYaml(const std::string& path) const
{
    // Cria um nó YAML vazio.
    YAML::Node node;

    // Armazena os pontos da imagem.
    node["pixel_points"] = pixel_points_;

    // Armazena os pontos reais.
    node["real_points"] = real_points_;

    // Abre o arquivo indicado para escrita.
    std::ofstream fout(path);

    // Escreve os dados no arquivo YAML.
    fout << node;

    // Fecha o arquivo.
    fout.close();

    // Informa que os dados foram salvos.
    std::cout << "[PixelConverter] Dados salvos em " << path << std::endl;
}

// ============================================================================
// Carrega os pontos de calibração de um arquivo YAML
// ============================================================================

void PixelConverter::loadFromYaml(const std::string& path)
{
    try
    {
        // Carrega o arquivo YAML especificado.
        YAML::Node node = YAML::LoadFile(path);

        // Recupera os pontos da imagem armazenados.
        pixel_points_ = node["pixel_points"].as<std::vector<cv::Point2f>>();

        // Recupera os pontos reais armazenados.
        real_points_ = node["real_points"].as<std::vector<cv::Point2f>>();

        // Recalcula a homografia utilizando os pontos carregados.
        H_ = cv::findHomography(pixel_points_, real_points_);

        // Calcula novamente sua inversa.
        H_inv_ = H_.inv();

        // Informa que os dados foram carregados corretamente.
        std::cout << "[PixelConverter] Dados carregados de " << path << std::endl;
    }
    catch (const std::exception& e)
    {
        // Caso ocorra qualquer erro na leitura do arquivo,
        // exibe a mensagem correspondente.
        std::cerr << "[PixelConverter] Erro ao carregar YAML: "
                  << e.what() << std::endl;
    }
}

// Final do namespace do projeto.
}  // namespace zenite_utils