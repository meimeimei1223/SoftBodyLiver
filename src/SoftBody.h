// SoftBody.h
#ifndef SOFT_BODY_H
#define SOFT_BODY_H

#include "VectorMath.h"
#include <vector>
#include <GL/glew.h>
#include <glm/glm.hpp>
#include <memory>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include "Hash.h"
#include <chrono>  // std::chrono を使うためにヘッダーを追加


class ShaderProgram;

class SoftBody {
public:
    struct MeshData {
        std::vector<float> verts;          // 頂点座標
        std::vector<int> tetIds;           // 四面体のインデックス
        std::vector<int> tetEdgeIds;       // エッジのインデックス
        std::vector<int> tetSurfaceTriIds; // 表面三角形のインデックス
    };

    // コンストラクタ
    SoftBody(const MeshData& meshData, float edgeCompliance = 100.0f, float volCompliance = 0.0f);
    ~SoftBody();

    // コンストラクタ
    SoftBody(const MeshData& meshData,const MeshData& vismeshData, float edgeCompliance = 100.0f, float volCompliance = 0.0f);

    SoftBody(const MeshData& tetMesh,
             const std::vector<MeshData>& visMeshes,
             float edgeCompliance = 100.0f,
             float volCompliance = 0.0f);

    // 物理シミュレーション
    void preSolve(float dt, const glm::vec3& gravity);
    void solve(float dt);
    void postSolve(float dt);
    void initPhysics();

    // 描画関連
    void draw(ShaderProgram& shader);
    void draw_Vis(ShaderProgram& shader);
    void drawTetMesh(ShaderProgram& shader);
    void updateMeshes();
    void updateTetMeshes();
    void updateVisMesh();
    void computeNormals();
    void computeVisNormals();


    // 変換
    void translate(float x, float y, float z);
    void squash();

    // インタラクション
    void startGrab(const glm::vec3& pos);
    void moveGrabbed(const glm::vec3& pos, const glm::vec3& vel);
    void endGrab(const glm::vec3& pos, const glm::vec3& vel);

    // アクセサ   std::vector<int> tetIds;
    const std::vector<float>& getPositions() const { return positions; }
    const std::vector<float>& getVisPositions() const { return vis_positions; }
    const std::vector<int>& getTetIds() const { return tetIds; }
    const std::vector<int>& gettetSurfaceTriIds() const { return tetSurfaceTriIds; }
    const std::vector<int>& getVisSurfaceTriIds() const { return visSurfaceTriIds; }

    const std::vector<float>& getVisPositions(int ids) const { return vis_positions_array[ids]; }
    const std::vector<int>& getVisSurfaceTriIds(int ids) const { return visSurfaceTriIds_array[ids]; }

    // const版 (読み取り専用)
    //const std::vector<std::vector<float>>& getVisPositionsArray() const { return vis_positions_array; }
    //const std::vector<std::vector<int>>& getVisSurfaceTriIdsArray() const { return visSurfaceTriIds_array; }


    size_t getNumParticles() const { return numParticles; }
    size_t getNumVisParticles() const { return numVisParticles; }
    void setEdgeCompliance(float compliance) { edgeCompliance = compliance; }
    void setVolCompliance(float compliance) { volCompliance = compliance; }
    // SoftBody.h のpublicセクションに追加
    const MeshData& getMeshData() const { return meshData; }
    const MeshData& getVisMeshData() const { return vismeshData; }
    const MeshData& getVisMeshData(int ids) const { return vismeshDataArray[ids]; }
    void applyShapeRestoration(float strength);

    std::vector<int> grabbedVertices;  // ★ここを追加

    const glm::mat4& getModelMatrix() const { return modelMatrix; }
    void setModelMatrix(const glm::mat4& matrix) { modelMatrix = matrix; }
    static MeshData loadTetMesh(const std::string& filename);
    static MeshData loadSurfaceMesh(const std::string& filename);


    bool isTetMeshVisible() const {
        return showTetMesh;
    }

    void computeSkinningInfo(const std::vector<float>& visVerts);
    const size_t& getNumVis() const { return numVisVerts; }

    int highlightedTriangle = -1;  // ヒットしたトライアングルのインデックス
    std::vector<int> highlightedTriangles;
    void setHighlightedTriangle(int index) { highlightedTriangle = index; }

    void clear();
    void initialize(const MeshData& tetMesh, const MeshData& visMesh, float edgeComp, float volComp);


    // Setup and update methods for multiple meshes
    void setupVisMeshes();
    void updateVisMeshes();
    void updateVisMeshes(const int correction);
    void computeSkinningInfoForMesh(const std::vector<float>& visVerts,
                                    std::vector<float>& skinningInfoOut);
    void computeVisNormalsForMesh(size_t meshIdx);
    void draw_Visarray(ShaderProgram& shader);
    void draw_AllVisMeshes(ShaderProgram& shader);
    void setActiveVisMesh(int index);
    void draw_AllVisMeshes(ShaderProgram& shader,glm::vec3 cameraPos);

    // ヘッダファイルに追加
    std::vector<float> edgeLambdas;  // エッジ拘束用のラグランジュ乗数
    std::vector<float> volLambdas;   // 体積拘束用のラグランジュ乗数

    // 既存のデータをクリア
private:
    // 追加のメンバー変数
    std::vector<int> activeParticles;  // 動く頂点のID
    const int NUM_ACTIVE_PARTICLES = 100;  // 動く頂点の数

    bool showTetMesh = true;
    glm::mat4 modelMatrix = glm::mat4(1.0f);
    // メッシュデータ
    const MeshData meshData;
    const MeshData vismeshData; // 表示用メッシュ
    size_t numParticles;
    size_t numTets;

    size_t numVisVerts;  // 表示用メッシュの頂点数
    std::vector<float> skinningInfo;  // スキニング情報

    // 物理シミュレーション用データ
    std::vector<float> positions;
    std::vector<float> prevPositions;
    std::vector<float> velocities;
    std::vector<int> tetIds;
    std::vector<int> tetSurfaceTriIds;
    std::vector<int> edgeIds;
    std::vector<float> restVols;
    std::vector<float> edgeLengths;
    std::vector<float> invMasses;
    std::vector<float> oldInvMasses;

    // 既存のメンバーに追加
    std::vector<float> vis_positions;  // 表示用メッシュの頂点位置
    std::vector<int> visSurfaceTriIds;
    size_t numVisParticles;
    // レンダリング用バッファ
    //GLuint VAO, VBO, EBO, normalVBO;
    std::vector<float> normalLines;
    //・GLuint tetVAO,tetVBO,tetEBO;
    // バッファハンドルの初期化
    GLuint VAO = 0;
    GLuint VBO = 0;
    GLuint EBO = 0;
    GLuint normalVBO = 0;
    GLuint tetVAO = 0;
    GLuint tetVBO = 0;
    GLuint tetEBO = 0;
    // 物理パラメータ
    float edgeCompliance;
    float volCompliance;
    float damping;

    // グラブ関連

    int grabId;
    float grabInvMass;

    // 一時バッファ
    std::vector<float> tempBuffer;
    std::vector<float> grads;

    // 四面体の頂点順序
    const std::vector<std::vector<int>> volIdOrder = {
        {1,3,2}, {0,2,3}, {0,3,1}, {0,1,2}
    };


    // Multiple visualization meshes data
    std::vector<MeshData> vismeshDataArray;

    // Arrays for each visualization mesh
    std::vector<std::vector<float>> vis_positions_array;
    std::vector<std::vector<int>> visSurfaceTriIds_array;
    std::vector<std::vector<float>> skinningInfo_array;

    // OpenGL data for each vis mesh
    std::vector<GLuint> visVAOs;
    std::vector<GLuint> visVBOs;
    std::vector<GLuint> visEBOs;
    std::vector<GLuint> visNormalVBOs;

    // Current active mesh index (for rendering)
    int activeVisMeshIndex = 0;

    // 空間ハッシュ関連
    struct TriData {
        size_t meshIdx;
        size_t triIdx;
        float distance;
    };

    // メソッドプロトタイプ
    uint64_t hashPosition(const glm::vec3& pos, float cellSize);
    void buildSpatialHash();

    // ヘルパー関数
    float getTetVolume(int nr);
    void solveEdges(float compliance, float dt);
    void solveVolumes(float compliance, float dt);
    void setupMesh(const std::vector<int>& surfaceTriIds);
    void setupVisMesh();
    void setupTetMesh();
    void setupNormalBuffer();
    void updateVisibility();
    void deleteBuffers();


    // 元のメッシュ形状を保存するための変数
    std::vector<std::vector<float>> original_vis_positions_array;  // 複数メッシュ用
    std::vector<std::vector<float>> vis_normals_array;
    // SoftBody.h の class SoftBody 定義内に追加
    // 複数メッシュ用の補正関数
    void buildAdjacencyListForMesh(size_t meshIdx, std::vector<std::vector<int>>& adjacencyList);
    void correctAbnormalVerticesForMesh(size_t meshIdx, const std::vector<bool>& isAbnormal,
                                        const std::vector<std::vector<int>>& adjacencyList);
    void applySmoothingForMesh(size_t meshIdx, int vertexIdx,
                               std::vector<float>& corrected_positions,
                               const std::vector<std::vector<int>>& adjacencyList);
    std::vector<std::vector<std::vector<int>>> cachedAdjacencyLists; // メッシュごとの隣接リストキャッシュ
    const float curvatureThreshold = 0.2f;     // 曲率の閾値
    const float maxNeighborDistance = 0.2f;    // 隣接点間の最大許容距離
    const float maxDeformationThreshold = 0.1f; // 最大変形量の閾値
    const int minValidNeighbors = 1;           // 必要な有効隣接点の最小数



};
#endif // SOFT_BODY_H
