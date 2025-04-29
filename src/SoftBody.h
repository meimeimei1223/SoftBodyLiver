// SoftBody.h
#ifndef SOFT_BODY_H
#define SOFT_BODY_H

#include <fstream>
#include <sstream>
#include "Hash.h"
#include "ShaderProgram.h"
#include "VectorMath.h"
#include "Neohookeanconstraint.h"

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
    SoftBody(const MeshData& meshData, const MeshData& vismeshData,
             float edgeCompliance = 100.0f, float volCompliance = 0.0f,
             float neoCompliance = 0.0f);  // NeohookeanのComplianceを追加

    // 物理シミュレーション
    void preSolve(float dt, const glm::vec3& gravity);
    void solve(float dt);
    void postSolve(float dt);
    void initPhysics();

    // 描画関連
    void draw_Vis(ShaderProgram& shader);
    void drawTetMesh(ShaderProgram& shader);
    void updateTetMeshes();
    void updateVisMesh();
    void computeNormals();
    void computeVisNormals();

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

    size_t getNumParticles() const { return numParticles; }
    size_t getNumVisParticles() const { return numVisParticles; }
    void setEdgeCompliance(float compliance) { edgeCompliance = compliance; }
    void setVolCompliance(float compliance) { volCompliance = compliance; }
    void setNeoCompliance(float compliance) { neoCompliance = compliance; }  // Neohookeanのコンプライアンス設定

    // SoftBody.h のpublicセクションに追加
    const MeshData& getMeshData() const { return meshData; }
    const MeshData& getVisMeshData() const { return vismeshData; }
    void applyShapeRestoration(float strength);

    std::vector<int> grabbedVertices;  // ★ここを追加

    const glm::mat4& getModelMatrix() const { return modelMatrix; }
    void setModelMatrix(const glm::mat4& matrix) { modelMatrix = matrix; }
    static MeshData loadTetMesh(const std::string& filename);
    static MeshData loadSurfaceMesh(const std::string& filename);

    void computeSkinningInfo(const std::vector<float>& visVerts);
    const size_t& getNumVis() const { return numVisVerts; }

    void clear();
    void initialize(const MeshData& tetMesh, const MeshData& visMesh,
                    float edgeComp, float volComp, float neoComp);  // Neohookeanのパラメータ追加

    // Setup and update methods for multiple meshes
    void setupVisMeshes();

    // ヘッダファイルに追加
    std::vector<float> edgeLambdas;  // エッジ拘束用のラグランジュ乗数
    std::vector<float> volLambdas;   // 体積拘束用のラグランジュ乗数
    std::vector<float> neoLambdas;   // Neohookean拘束用のラグランジュ乗数

    // Neohookean物性パラメータの設定
    void setMaterialParameters(float youngModulus, float poissonRatio);

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

    // Neohookean制約オブジェクト
    std::vector<NeohookeanConstraint> neoConstraints;
    float youngModulus = 1000.0f;  // ヤング率のデフォルト値
    float poissonRatio = 0.45f;    // ポアソン比のデフォルト値

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
    float neoCompliance;  // Neohookean用のコンプライアンス
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

    // メソッドプロトタイプ
    uint64_t hashPosition(const glm::vec3& pos, float cellSize);

    // ヘルパー関数
    float getTetVolume(int nr);
    void solveEdges(float compliance, float dt);
    void solveVolumes(float compliance, float dt);
    void solveNeohookean(float compliance, float dt);  // Neohookean制約を解く関数
    void setupVisMesh();
    void setupTetMesh();
    void setupNormalBuffer();
    void deleteBuffers();
    void initNeohookeanConstraints();  // Neohookean制約の初期化
};
#endif // SOFT_BODY_H
