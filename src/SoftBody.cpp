// SoftBody.cpp
#include "SoftBody.h"
#include "VectorMath.h"
#include <iostream>
#include "ShaderProgram.h"

SoftBody::SoftBody(const MeshData& meshData, float edgeCompliance, float volCompliance)
    : meshData(meshData)
    , edgeCompliance(edgeCompliance)
    , volCompliance(volCompliance)
    , grabId(-1)
    , grabInvMass(0.0f)
    , damping(0.99f)
{
    // メッシュデータの初期化
    numParticles = meshData.verts.size() / 3;
    numTets = meshData.tetIds.size() / 4;

    positions = meshData.verts;
    prevPositions = meshData.verts;
    velocities.resize(3 * numParticles, 0.0f);

    tetIds = meshData.tetIds;
    edgeIds = meshData.tetEdgeIds;
    restVols.resize(numTets, 0.0f);
    edgeLengths.resize(edgeIds.size() / 2, 0.0f);
    invMasses.resize(numParticles, 0.0f);

    tempBuffer.resize(4 * 3, 0.0f);
    grads.resize(4 * 3, 0.0f);

    edgeLambdas.resize(edgeIds.size() / 2, 0.0f);
    volLambdas.resize(numTets, 0.0f);

    initPhysics();
    setupMesh(meshData.tetSurfaceTriIds);
    setupNormalBuffer();
}

SoftBody::SoftBody(const MeshData& tetMesh, const MeshData& visMesh, float edgeCompliance, float volCompliance)
    : meshData(tetMesh)  // 物理シミュレーション用メッシュを保存
    , vismeshData(visMesh)
    , edgeCompliance(edgeCompliance)
    , volCompliance(volCompliance)
    , grabId(-1)
    , grabInvMass(0.0f)
    , damping(0.99f)
{
    // デバッグ出力の追加
    std::cout << "=== SoftBody Constructor Debug ===" << std::endl;

    // TetMeshの情報
    std::cout << "TetMesh Info:" << std::endl;
    std::cout << "  Vertices: " << tetMesh.verts.size() / 3 << std::endl;
    std::cout << "  Tetrahedra: " << tetMesh.tetIds.size() / 4 << std::endl;
    std::cout << "  Edges: " << tetMesh.tetEdgeIds.size() / 2 << std::endl;
    std::cout << "  Surface Triangles: " << tetMesh.tetSurfaceTriIds.size() / 3 << std::endl;

    // VisMeshの情報
    std::cout << "VisMesh Info:" << std::endl;
    std::cout << "  Vertices: " << visMesh.verts.size() / 3 << std::endl;
    std::cout << "  Surface Triangles: " << visMesh.tetSurfaceTriIds.size() / 3 << std::endl;

    // インデックスの範囲チェック
    size_t maxTetIndex = 0;
    for (const auto& id : tetMesh.tetIds) {
        maxTetIndex = std::max(maxTetIndex, static_cast<size_t>(id));
    }
    std::cout << "Max Tet Index: " << maxTetIndex << " (should be < " << tetMesh.verts.size() / 3 << ")" << std::endl;

    // バウンディングボックスの確認
    glm::vec3 tetMin(std::numeric_limits<float>::max());
    glm::vec3 tetMax(std::numeric_limits<float>::lowest());
    glm::vec3 visMin(std::numeric_limits<float>::max());
    glm::vec3 visMax(std::numeric_limits<float>::lowest());

    // TetMeshのバウンディングボックス計算
    for (size_t i = 0; i < tetMesh.verts.size(); i += 3) {
        tetMin.x = std::min(tetMin.x, tetMesh.verts[i]);
        tetMin.y = std::min(tetMin.y, tetMesh.verts[i + 1]);
        tetMin.z = std::min(tetMin.z, tetMesh.verts[i + 2]);
        tetMax.x = std::max(tetMax.x, tetMesh.verts[i]);
        tetMax.y = std::max(tetMax.y, tetMesh.verts[i + 1]);
        tetMax.z = std::max(tetMax.z, tetMesh.verts[i + 2]);
    }

    // VisMeshのバウンディングボックス計算
    for (size_t i = 0; i < visMesh.verts.size(); i += 3) {
        visMin.x = std::min(visMin.x, visMesh.verts[i]);
        visMin.y = std::min(visMin.y, visMesh.verts[i + 1]);
        visMin.z = std::min(visMin.z, visMesh.verts[i + 2]);
        visMax.x = std::max(visMax.x, visMesh.verts[i]);
        visMax.y = std::max(visMax.y, visMesh.verts[i + 1]);
        visMax.z = std::max(visMax.z, visMesh.verts[i + 2]);
    }

    std::cout << "TetMesh Bounds: " << std::endl;
    std::cout << "  Min: (" << tetMin.x << ", " << tetMin.y << ", " << tetMin.z << ")" << std::endl;
    std::cout << "  Max: (" << tetMax.x << ", " << tetMax.y << ", " << tetMax.z << ")" << std::endl;
    std::cout << "VisMesh Bounds: " << std::endl;
    std::cout << "  Min: (" << visMin.x << ", " << visMin.y << ", " << visMin.z << ")" << std::endl;
    std::cout << "  Max: (" << visMax.x << ", " << visMax.y << ", " << visMax.z << ")" << std::endl;

    std::cout << "=== Constructor Debug End ===" << std::endl;

    // 物理シミュレーション用メッシュの初期化
    std::cout << "=== 物理シミュレーション用メッシュの初期化 ===" << std::endl;
    numParticles = tetMesh.verts.size() / 3;
    numTets = tetMesh.tetIds.size() / 4;

    numVisVerts = visMesh.verts.size() / 3;  // 表示用メッシュの頂点数
    vis_positions = visMesh.verts;
    numVisParticles = visMesh.verts.size() / 3;
    visSurfaceTriIds = visMesh.tetSurfaceTriIds;

    // 物理シミュレーション用データの初期化
    std::cout << "=== 物理シミュレーション用データの初期化 ===" << std::endl;
    positions = tetMesh.verts;
    prevPositions = tetMesh.verts;
    velocities.resize(3 * numParticles, 0.0f);

    tetIds = tetMesh.tetIds;
    tetSurfaceTriIds = tetMesh.tetSurfaceTriIds;
    edgeIds = tetMesh.tetEdgeIds;

    restVols.resize(numTets, 0.0f);
    edgeLengths.resize(edgeIds.size() / 2, 0.0f);
    invMasses.resize(numParticles, 0.0f);

    edgeLambdas.resize(edgeIds.size() / 2, 0.0f);
    volLambdas.resize(numTets, 0.0f);
    // 一時バッファの初期化
    std::cout << "=== 一時バッファの初期化 ===" << std::endl;
    tempBuffer.resize(4 * 3, 0.0f);
    grads.resize(4 * 3, 0.0f);

    // スキニング情報の初期化
    std::cout << "=== スキニング情報の初期化1 ===" << std::endl;
    skinningInfo.resize(4 * numVisVerts, -1.0f);
    std::cout << "=== スキニング情報の初期化2 ===" << std::endl;
    computeSkinningInfo(visMesh.verts);

    // 物理パラメータの初期化
    std::cout << "=== 物理パラメータの初期化 ===" << std::endl;
    initPhysics();

    // 表示用メッシュのセットアップ
    std::cout << "=== 表示用メッシュのセットアップ ===" << std::endl;
    setupVisMesh();  // 表示用メッシュの三角形を使用

    // 四面体メッシュの表示設定
    std::cout << "=== 四面体メッシュの表示設定 ===" << std::endl;
    setupTetMesh();  // ★これを追加

    // 法線バッファの初期化
    std::cout << "=== 法線バッファの初期化===" << std::endl;
    setupNormalBuffer();

    // 初期状態では四面体メッシュは非表示
    std::cout << "=== 初期状態では四面体メッシュは非表示===" << std::endl;
    showTetMesh = true;

    // モデル行列の初期化
    std::cout << "=== モデル行列の初期化 ===" << std::endl;
    modelMatrix = glm::mat4(1.0f);
}

void SoftBody::computeSkinningInfo(const std::vector<float>& visVerts) {
    // 頂点用のハッシュを作成
    //std::cout << "=== 頂点用のハッシュを作成===" << std::endl;
    std::cout << "Number of visual vertices: " << numVisVerts << std::endl;
    // バウンディングボックスの計算
    glm::vec3 tetMin(std::numeric_limits<float>::max());
    glm::vec3 tetMax(std::numeric_limits<float>::lowest());
    glm::vec3 visMin(std::numeric_limits<float>::max());
    glm::vec3 visMax(std::numeric_limits<float>::lowest());

    // 四面体メッシュのバウンディングボックス
    for (size_t i = 0; i < positions.size(); i += 3) {
        tetMin.x = std::min(tetMin.x, positions[i]);
        tetMin.y = std::min(tetMin.y, positions[i + 1]);
        tetMin.z = std::min(tetMin.z, positions[i + 2]);
        tetMax.x = std::max(tetMax.x, positions[i]);
        tetMax.y = std::max(tetMax.y, positions[i + 1]);
        tetMax.z = std::max(tetMax.z, positions[i + 2]);
    }

    // 表示用メッシュのバウンディングボックス
    for (size_t i = 0; i < visVerts.size(); i += 3) {
        visMin.x = std::min(visMin.x, visVerts[i]);
        visMin.y = std::min(visMin.y, visVerts[i + 1]);
        visMin.z = std::min(visMin.z, visVerts[i + 2]);
        visMax.x = std::max(visMax.x, visVerts[i]);
        visMax.y = std::max(visMax.y, visVerts[i + 1]);
        visMax.z = std::max(visMax.z, visVerts[i + 2]);
    }

    // スケール係数の計算
    glm::vec3 tetSize = tetMax - tetMin;
    glm::vec3 visSize = visMax - visMin;
    float maxSize = std::max({tetSize.x, tetSize.y, tetSize.z});
    float spacing = maxSize * 1.0f;  // メッシュサイズに応じたスペーシング
    std::cout << "rMax = std::min(rMax + spacing, maxSize * 0.5f);" << std::endl;
    std::cout << "spacing;" << spacing << std::endl;
    std::cout << "maxSize;" << maxSize << std::endl;
    std::cout << "Hash hash(spacing, numVisVerts);" << std::endl;
    Hash hash(spacing, numVisVerts);
    hash.create(visVerts);

    // スキニング情報を初期化
    //std::cout << "=== スキニング情報を初期化 ===" << std::endl;
    skinningInfo.assign(4 * numVisVerts, -1.0f);
    std::vector<float> minDist(numVisVerts, std::numeric_limits<float>::max());
    const float border = 0.05f;

    // 四面体の中心と行列用の一時バッファ
    //std::cout << "=== 四面体の中心と行列用の一時バッファ ===" << std::endl;
    std::vector<float> tetCenter(3, 0.0f);
    std::vector<float> mat(9, 0.0f);
    std::vector<float> bary(4, 0.0f);

    // 各四面体について処理
    //std::cout << "=== 各四面体について処理 ===" << std::endl;
    for (size_t i = 0; i < numTets; i++) {
        // 四面体の中心を計算
        std::fill(tetCenter.begin(), tetCenter.end(), 0.0f);
        for (int j = 0; j < 4; j++) {
            VectorMath::vecAdd(tetCenter, 0, positions, tetIds[4 * i + j], 0.25f);
        }
        // バウンディングスフィアの半径を計算
        //std::cout << "=== バウンディングスフィアの半径を計算===" << std::endl;
        float rMax = 0.0f;
        for (int j = 0; j < 4; j++) {
            float r2 = VectorMath::vecDistSquared(tetCenter, 0, positions, tetIds[4 * i + j]);
            rMax = std::max(rMax, std::sqrt(r2));
        }
        rMax += border;

        // 近傍頂点を検索
        //std::cout << "=== 近傍頂点を検索 ===" << std::endl;
        hash.query(tetCenter, 0, rMax);
        if (hash.querySize == 0) continue;

        // 四面体の頂点インデックス
        //std::cout << "===四面体の頂点インデックス===" << std::endl;
        int id0 = tetIds[4 * i];
        int id1 = tetIds[4 * i + 1];
        int id2 = tetIds[4 * i + 2];
        int id3 = tetIds[4 * i + 3];

        // 行列を計算
        //std::cout << "=== 行列を計算 ===" << std::endl;
        VectorMath::vecSetDiff(mat, 0, positions, id0, positions, id3);
        VectorMath::vecSetDiff(mat, 1, positions, id1, positions, id3);
        VectorMath::vecSetDiff(mat, 2, positions, id2, positions, id3);
        VectorMath::matSetInverse(mat);

        // 検索された各頂点に対して処理
        //std::cout << "=== 検索された各頂点に対して処理 ===" << std::endl;
        for (int j = 0; j < hash.querySize; j++) {
            int id = hash.queryIds[j];

            // すでにスキニング情報がある場合はスキップ
            //std::cout << "=== すでにスキニング情報がある場合はスキップ ===" << std::endl;
            //std::cout << "minDist[id]: " << minDist[id]  << std::endl;
            if (minDist[id] <= 0.0f) continue;

            // 距離チェック
            //std::cout << "=== 距離チェック ===" << std::endl;
            if (VectorMath::vecDistSquared(visVerts, id, tetCenter, 0) > rMax * rMax) continue;

            // 重心座標を計算
            //std::cout << "=== 重心座標を計算==" << std::endl;
            VectorMath::vecSetDiff(bary, 0, visVerts, id, positions, id3);
            VectorMath::matSetMult(mat, bary, 0, bary, 0);
            bary[3] = 1.0f - bary[0] - bary[1] - bary[2];

            // 最大距離を計算
            //std::cout << "=== 最大距離を計算 ===" << std::endl;
            float dist = 0.0f;
            for (int k = 0; k < 4; k++) {
                dist = std::max(dist, -bary[k]);
            }

            // より良い結果が見つかった場合は更新
            //std::cout << "=== より良い結果が見つかった場合は更新 ===" << std::endl;
            if (dist < minDist[id]) {
                minDist[id] = dist;
                skinningInfo[4 * id] = static_cast<float>(i);
                skinningInfo[4 * id + 1] = bary[0];
                skinningInfo[4 * id + 2] = bary[1];
                skinningInfo[4 * id + 3] = bary[2];
            }
        }
    }
}


float SoftBody::getTetVolume(int nr) {
    int id0 = tetIds[4 * nr];
    int id1 = tetIds[4 * nr + 1];
    int id2 = tetIds[4 * nr + 2];
    int id3 = tetIds[4 * nr + 3];

    VectorMath::vecSetDiff(tempBuffer, 0, positions, id1, positions, id0);
    VectorMath::vecSetDiff(tempBuffer, 1, positions, id2, positions, id0);
    VectorMath::vecSetDiff(tempBuffer, 2, positions, id3, positions, id0);
    VectorMath::vecSetCross(tempBuffer, 3, tempBuffer, 0, tempBuffer, 1);

    return VectorMath::vecDot(tempBuffer, 3, tempBuffer, 2) / 6.0f;
}

// 3. initPhysics() 関数内でラグランジュ乗数のリセットを追加
void SoftBody::initPhysics() {
    std::fill(invMasses.begin(), invMasses.end(), 0.0f);
    std::fill(restVols.begin(), restVols.end(), 0.0f);
    std::fill(edgeLambdas.begin(), edgeLambdas.end(), 0.0f);  // ラグランジュ乗数をリセット
    std::fill(volLambdas.begin(), volLambdas.end(), 0.0f);    // ラグランジュ乗数をリセット

    // 四面体の質量を計算
    float cusum_vol = 1.0;
    for (size_t i = 0; i < numTets; i++) {
        float vol = getTetVolume(i);
        restVols[i] = vol;
        float pInvMass = vol > 0.0f ? 1.0f / (vol / 4000000.0f) : 1000000.0f;

        for (int j = 0; j < 4; j++) {
            invMasses[tetIds[4 * i + j]] += pInvMass;
        }
    }

    // エッジの初期長さを計算
    for (size_t i = 0; i < edgeLengths.size(); i++) {
        int id0 = edgeIds[2 * i];
        int id1 = edgeIds[2 * i + 1];
        edgeLengths[i] = std::sqrt(VectorMath::vecDistSquared(positions, id0, positions, id1));
    }
}

void SoftBody::solve(float dt) {
    // 複数回の反復でソルバーの安定性を高める
    for (int i = 0; i < 5; i++) {
        solveEdges(edgeCompliance, dt);
        // 体積拘束は回数を増やして安定性を確保
        for (int j = 0; j < 2; j++) {
            solveVolumes(volCompliance, dt);
        }
    }
}

// 5. solveEdges() 関数をXPBD用に書き換え
void SoftBody::solveEdges(float compliance, float dt) {
    // XPBDでは、コンプライアンスはdt依存
    float alpha = compliance / (dt * dt);

    for (size_t i = 0; i < edgeLengths.size(); i++) {
        int id0 = edgeIds[2 * i];
        int id1 = edgeIds[2 * i + 1];
        float w0 = invMasses[id0];
        float w1 = invMasses[id1];
        float w = w0 + w1;
        if (w == 0.0f) continue;

        VectorMath::vecSetDiff(grads, 0, positions, id0, positions, id1);
        float len = std::sqrt(VectorMath::vecLengthSquared(grads, 0));
        if (len == 0.0f) continue;

        VectorMath::vecScale(grads, 0, 1.0f / len);
        float restLen = edgeLengths[i];
        float C = len - restLen;

        // XPBDの核心部分：ラグランジュ乗数の更新
        float dLambda = -(C + alpha * edgeLambdas[i]) / (w + alpha);
        edgeLambdas[i] += dLambda;

        // 拘束の適用
        VectorMath::vecAdd(positions, id0, grads, 0, dLambda * w0);
        VectorMath::vecAdd(positions, id1, grads, 0, -dLambda * w1);
    }
}

// 6. solveVolumes() 関数をXPBD用に書き換え
void SoftBody::solveVolumes(float compliance, float dt) {
    // XPBDでは、コンプライアンスはdt依存
    float alpha = compliance / (dt * dt);

    for (size_t i = 0; i < numTets; i++) {
        float w = 0.0f;

        // 四面体の各頂点に対する勾配を計算
        for (int j = 0; j < 4; j++) {
            int id0 = tetIds[4 * i + volIdOrder[j][0]];
            int id1 = tetIds[4 * i + volIdOrder[j][1]];
            int id2 = tetIds[4 * i + volIdOrder[j][2]];

            VectorMath::vecSetDiff(tempBuffer, 0, positions, id1, positions, id0);
            VectorMath::vecSetDiff(tempBuffer, 1, positions, id2, positions, id0);
            VectorMath::vecSetCross(grads, j, tempBuffer, 0, tempBuffer, 1);
            VectorMath::vecScale(grads, j, 1.0f/6.0f);

            w += invMasses[tetIds[4 * i + j]] * VectorMath::vecLengthSquared(grads, j);
        }

        if (w == 0.0f) continue;

        float vol = getTetVolume(i);
        float restVol = restVols[i];
        float C = vol - restVol;

        // XPBDの核心部分：ラグランジュ乗数の更新
        float dLambda = -(C + alpha * volLambdas[i]) / (w + alpha);
        volLambdas[i] += dLambda;

        // 位置の修正を適用
        for (int j = 0; j < 4; j++) {
            int id = tetIds[4 * i + j];
            VectorMath::vecAdd(positions, id, grads, j, dLambda * invMasses[id]);
        }
    }
}

void SoftBody::preSolve(float dt, const glm::vec3& gravity) {
    for (size_t i = 0; i < numParticles; i++) {
        if (invMasses[i] == 0.0f) continue;

        // 速度に重力を適用
        VectorMath::vecAdd(velocities, i, {gravity.x, gravity.y, gravity.z}, 0, dt);

        // 減衰を適用
        VectorMath::vecScale(velocities, i, damping);

        // 位置を更新
        VectorMath::vecCopy(prevPositions, i, positions, i);
        VectorMath::vecAdd(positions, i, velocities, i, dt);

        // 地面との衝突処理
        if (positions[3 * i + 1] < -2.0f) {
            VectorMath::vecCopy(positions, i, prevPositions, i);
            positions[3 * i + 1] = -2.0f;
            velocities[3 * i + 1] = -2.0f;  // Y方向の速度をリセット
        }
    }
}

void SoftBody::postSolve(float dt) {
    for (size_t i = 0; i < numParticles; i++) {
        if (invMasses[i] == 0.0f) continue;
        VectorMath::vecSetDiff(velocities, i, positions, i, prevPositions, i, 1.0f / dt);
    }
}

void SoftBody::setupMesh(const std::vector<int>& surfaceTriIds) {
    // Delete any existing buffers first
    deleteBuffers();
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);
    glGenBuffers(1, &normalVBO);

    glBindVertexArray(VAO);

    // 頂点位置
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, positions.size() * sizeof(float),
                 positions.data(), GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // 法線
    glBindBuffer(GL_ARRAY_BUFFER, normalVBO);
    computeNormals();
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);

    // インデックス
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 surfaceTriIds.size() * sizeof(int),
                 surfaceTriIds.data(), GL_STATIC_DRAW);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void SoftBody::updateMeshes() {
    // 頂点位置の更新
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0,
                    positions.size() * sizeof(float), positions.data());
    // 法線の更新
    computeNormals();
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void SoftBody::draw(ShaderProgram& shader) {
    shader.use();
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, meshData.tetSurfaceTriIds.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}

void SoftBody::computeNormals() {
    std::vector<glm::vec3> normals(numParticles, glm::vec3(0.0f));

    // 面ごとに法線を計算して蓄積
    for (size_t i = 0; i < meshData.tetSurfaceTriIds.size(); i += 3) {
        int id0 = meshData.tetSurfaceTriIds[i];
        int id1 = meshData.tetSurfaceTriIds[i + 1];
        int id2 = meshData.tetSurfaceTriIds[i + 2];

        glm::vec3 p0(positions[id0 * 3], positions[id0 * 3 + 1], positions[id0 * 3 + 2]);
        glm::vec3 p1(positions[id1 * 3], positions[id1 * 3 + 1], positions[id1 * 3 + 2]);
        glm::vec3 p2(positions[id2 * 3], positions[id2 * 3 + 1], positions[id2 * 3 + 2]);

        glm::vec3 normal = glm::normalize(glm::cross(p1 - p0, p2 - p0));

        normals[id0] += normal;
        normals[id1] += normal;
        normals[id2] += normal;
    }

    // 法線を正規化してバッファに格納
    std::vector<float> normalBuffer;
    normalBuffer.reserve(numParticles * 3);

    for (const auto& n : normals) {
        glm::vec3 normalized = glm::length(n) > 0.0f ? glm::normalize(n) : n;
        normalBuffer.push_back(normalized.x);
        normalBuffer.push_back(normalized.y);
        normalBuffer.push_back(normalized.z);
    }

    // 法線バッファを更新
    glBindBuffer(GL_ARRAY_BUFFER, normalVBO);
    glBufferData(GL_ARRAY_BUFFER, normalBuffer.size() * sizeof(float),
                 normalBuffer.data(), GL_DYNAMIC_DRAW);
}



void SoftBody::setupVisMesh() {
    deleteBuffers();
    // バッファの生成
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
    glGenBuffers(1, &EBO);
    glGenBuffers(1, &normalVBO);

    glBindVertexArray(VAO);

    // 表示用頂点バッファの初期化
    vis_positions.resize(3 * numVisVerts, 0.0f);

    // 頂点位置
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vis_positions.size() * sizeof(float),
                 vis_positions.data(), GL_DYNAMIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);
    // 法線
    glBindBuffer(GL_ARRAY_BUFFER, normalVBO);
    computeVisNormals();
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);

    // インデックス
    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, EBO);
    glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                 vismeshData.tetSurfaceTriIds.size() * sizeof(int),
                 vismeshData.tetSurfaceTriIds.data(), GL_STATIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void SoftBody::computeVisNormals() {
    std::vector<glm::vec3> normals(numVisVerts, glm::vec3(0.0f));  // numParticles → numVisVerts

    // 面ごとに法線を計算して蓄積
    for (size_t i = 0; i < vismeshData.tetSurfaceTriIds.size(); i += 3) {  // meshData → vismeshData
        int id0 = vismeshData.tetSurfaceTriIds[i];
        int id1 = vismeshData.tetSurfaceTriIds[i + 1];
        int id2 = vismeshData.tetSurfaceTriIds[i + 2];

        glm::vec3 p0(vis_positions[id0 * 3], vis_positions[id0 * 3 + 1], vis_positions[id0 * 3 + 2]);  // positions → vis_positions
        glm::vec3 p1(vis_positions[id1 * 3], vis_positions[id1 * 3 + 1], vis_positions[id1 * 3 + 2]);
        glm::vec3 p2(vis_positions[id2 * 3], vis_positions[id2 * 3 + 1], vis_positions[id2 * 3 + 2]);

        glm::vec3 normal = glm::normalize(glm::cross(p1 - p0, p2 - p0));

        normals[id0] += normal;
        normals[id1] += normal;
        normals[id2] += normal;
    }

    // 法線を正規化してバッファに格納
    std::vector<float> normalBuffer;
    normalBuffer.reserve(numVisVerts * 3);  // numParticles → numVisVerts

    for (const auto& n : normals) {
        glm::vec3 normalized = glm::length(n) > 0.0f ? glm::normalize(n) : n;
        normalBuffer.push_back(normalized.x);
        normalBuffer.push_back(normalized.y);
        normalBuffer.push_back(normalized.z);
    }

    // 法線バッファを更新
    glBindBuffer(GL_ARRAY_BUFFER, normalVBO);
    glBufferData(GL_ARRAY_BUFFER, normalBuffer.size() * sizeof(float),
                 normalBuffer.data(), GL_DYNAMIC_DRAW);
}

void SoftBody::updateVisMesh() {
    // 表示用メッシュの頂点位置を更新
    vis_positions.resize(3 * numVisVerts, 0.0f);

    int nr = 0;
    for (size_t i = 0; i < numVisVerts; i++) {
        // 四面体のインデックスを取得
        int tetNr = static_cast<int>(skinningInfo[nr++]) * 4;
        if (tetNr < 0) {
            nr += 3;  // スキニング情報をスキップ
            continue;
        }

        // バリセントリック座標（重み）を取得
        float b0 = skinningInfo[nr++];
        float b1 = skinningInfo[nr++];
        float b2 = skinningInfo[nr++];
        float b3 = 1.0f - b0 - b1 - b2;

        // 四面体の頂点インデックスを取得
        int id0 = tetIds[tetNr++];
        int id1 = tetIds[tetNr++];
        int id2 = tetIds[tetNr++];
        int id3 = tetIds[tetNr++];

        // 頂点位置を初期化
        VectorMath::vecSetZero(vis_positions, i);

        // 重み付きで四面体の各頂点の影響を加算
        VectorMath::vecAdd(vis_positions, i, positions, id0, b0);
        VectorMath::vecAdd(vis_positions, i, positions, id1, b1);
        VectorMath::vecAdd(vis_positions, i, positions, id2, b2);
        VectorMath::vecAdd(vis_positions, i, positions, id3, b3);
    }

    // OpenGLバッファを更新
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0,
                    vis_positions.size() * sizeof(float),
                    vis_positions.data());

    // 法線を再計算
    computeVisNormals();

    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void SoftBody::draw_Vis(ShaderProgram& shader) {
    shader.use();
    glBindVertexArray(VAO);
    glDrawElements(GL_TRIANGLES, vismeshData.tetSurfaceTriIds.size(), GL_UNSIGNED_INT, 0);
    glBindVertexArray(0);
}


void SoftBody::setupTetMesh() {
    // Delete any existing tet mesh buffers
    if (tetVAO != 0) {
        glDeleteVertexArrays(1, &tetVAO);
        tetVAO = 0;
    }
    if (tetVBO != 0) {
        glDeleteBuffers(1, &tetVBO);
        tetVBO = 0;
    }
    if (tetEBO != 0) {
        glDeleteBuffers(1, &tetEBO);
        tetEBO = 0;
    }
    // VAOとVBOの生成
    glGenVertexArrays(1, &tetVAO);
    glGenBuffers(1, &tetVBO);
    glGenBuffers(1, &tetEBO);

    glBindVertexArray(tetVAO);

    // エッジ頂点データの作成
    std::vector<float> edgeVertices;
    for (size_t i = 0; i < meshData.tetEdgeIds.size(); i += 2) {
        int id0 = meshData.tetEdgeIds[i];
        int id1 = meshData.tetEdgeIds[i + 1];

        // 最初の頂点
        edgeVertices.push_back(positions[id0 * 3]);
        edgeVertices.push_back(positions[id0 * 3 + 1]);
        edgeVertices.push_back(positions[id0 * 3 + 2]);

        // 二番目の頂点
        edgeVertices.push_back(positions[id1 * 3]);
        edgeVertices.push_back(positions[id1 * 3 + 1]);
        edgeVertices.push_back(positions[id1 * 3 + 2]);
    }

    // 頂点バッファの設定
    glBindBuffer(GL_ARRAY_BUFFER, tetVBO);
    glBufferData(GL_ARRAY_BUFFER, edgeVertices.size() * sizeof(float),
                 edgeVertices.data(), GL_DYNAMIC_DRAW);

    // 頂点属性の設定
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void SoftBody::updateTetMeshes() {
    std::vector<float> edgeVertices;
    for (size_t i = 0; i < meshData.tetEdgeIds.size(); i += 2) {
        int id0 = meshData.tetEdgeIds[i];
        int id1 = meshData.tetEdgeIds[i + 1];

        edgeVertices.push_back(positions[id0 * 3]);
        edgeVertices.push_back(positions[id0 * 3 + 1]);
        edgeVertices.push_back(positions[id0 * 3 + 2]);

        edgeVertices.push_back(positions[id1 * 3]);
        edgeVertices.push_back(positions[id1 * 3 + 1]);
        edgeVertices.push_back(positions[id1 * 3 + 2]);
    }

    glBindBuffer(GL_ARRAY_BUFFER, tetVBO);
    glBufferData(GL_ARRAY_BUFFER, edgeVertices.size() * sizeof(float),
                 edgeVertices.data(), GL_DYNAMIC_DRAW);
    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

void SoftBody::drawTetMesh(ShaderProgram& shader) {
    if (!showTetMesh) return;

    shader.use();
    glBindVertexArray(tetVAO);

    // ワイヤーフレームモードで描画
    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);

    // エッジの数の2倍（各エッジの両端点）を頂点数として指定
    glDrawArrays(GL_LINES, 0, meshData.tetEdgeIds.size());

    // 元のポリゴンモードに戻す
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    glBindVertexArray(0);
}



void SoftBody::setupNormalBuffer() {
    // 法線表示用のバッファ初期化
    normalLines.clear();
    computeNormals();
}


void SoftBody::startGrab(const glm::vec3& pos) {
    //initPhysics(); //重心の固定をする

    float minD2 = std::numeric_limits<float>::max();
    grabId = -1;

    struct ParticleDistance {
        int id;
        float distance;
    };

    std::vector<ParticleDistance> sortedParticles;
    // アクティブな頂点をリセット
    activeParticles.clear();
    // 最も近い頂点を探す
    for (size_t i = 0; i < numParticles; i++) {
        glm::vec3 particlePos(positions[i * 3], positions[i * 3 + 1], positions[i * 3 + 2]);
        glm::vec3 diff = particlePos - pos;
        float d2 = glm::dot(diff, diff);  // distance squared
        if (d2 < minD2) {
            minD2 = d2;
            grabId = i;
        }
        sortedParticles.push_back({(int)i, d2});
    }

    // 距離でソート
    std::sort(sortedParticles.begin(), sortedParticles.end(),
              [](const ParticleDistance& a, const ParticleDistance& b) {
                  return a.distance < b.distance;
              });

    // 最も近い NUM_ACTIVE_PARTICLES 個を選択
    //int numToSelect = std::min(NUM_ACTIVE_PARTICLES,
    //                        static_cast<int>(sortedParticles.size()));
    // 頂点数の30%を選択
    int numToSelect = static_cast<int>(sortedParticles.size() * 1.0);
    // 最小値を確保（少なくとも1つの頂点は選択する）
    numToSelect = std::max(1, numToSelect);
    std::cout << "numToSelect: " << numToSelect  << std::endl;
    std::cout << "sortedParticles.size(): " << sortedParticles.size()  << std::endl;

    for (int i = 0; i < numToSelect; i++) {
        activeParticles.push_back(sortedParticles[i].id);
    }

    // すべての頂点を固定
    oldInvMasses = invMasses;  // 元の質量を保存


    // すべての頂点を固定
    std::fill(invMasses.begin(), invMasses.end(), 0.0f);

    // アクティブな頂点の質量を戻す
    for (int id : activeParticles) {
        invMasses[id] = oldInvMasses[id];
    }

    // 掴んだ頂点の設定
    invMasses[grabId] = 0.0f;

    int last = numParticles -1;
    //invMasses[last] = 0.0f;

    // 頂点を指定位置に移動
    positions[grabId * 3] = pos.x;
    positions[grabId * 3 + 1] = pos.y;
    positions[grabId * 3 + 2] = pos.z;
}


void SoftBody::moveGrabbed(const glm::vec3& pos, const glm::vec3& vel) {
    if (grabId >= 0) {
        // 掴んだ頂点を新しい位置に移動
        positions[grabId * 3] = pos.x;
        positions[grabId * 3 + 1] = pos.y;
        positions[grabId * 3 + 2] = pos.z;
    }
}

void SoftBody::endGrab(const glm::vec3& pos, const glm::vec3& vel) {
    if (grabId >= 0) {
        // アクティブな頂点だけ元の質量に戻す
        for (int id : activeParticles) {
            //if (id != grabId) {  // grabId以外の頂点のみ元に戻す
                invMasses[id] = oldInvMasses[id];
            //}
        }

        // grabIdの頂点の質量は0のまま維持
        //invMasses[grabId] = 0.0f;  // 引き続き固定

        // 掴んでいた頂点の速度を設定
        velocities[grabId * 3] = vel.x;
        velocities[grabId * 3 + 1] = vel.y;
        velocities[grabId * 3 + 2] = vel.z;

        grabId = -1;
        activeParticles.clear();
    }
}



void SoftBody::applyShapeRestoration(float strength) {
    for (size_t i = 0; i < numParticles; i++) {
        glm::vec3 restPos(meshData.verts[i * 3], meshData.verts[i * 3 + 1], meshData.verts[i * 3 + 2]);
        glm::vec3 currentPos(positions[i * 3], positions[i * 3 + 1], positions[i * 3 + 2]);

        glm::vec3 correction = (restPos - currentPos) * strength;

        positions[i * 3] += correction.x;
        positions[i * 3 + 1] += correction.y;
        positions[i * 3 + 2] += correction.z;
    }

    for (size_t i = 0; i < numVisParticles; i++) {
        glm::vec3 restPos(vismeshData.verts[i * 3], vismeshData.verts[i * 3 + 1], vismeshData.verts[i * 3 + 2]);
        glm::vec3 currentPos(vis_positions[i * 3], vis_positions[i * 3 + 1], vis_positions[i * 3 + 2]);

        glm::vec3 correction = (restPos - currentPos) * strength;

        vis_positions[i * 3] += correction.x;
        vis_positions[i * 3 + 1] += correction.y;
        vis_positions[i * 3 + 2] += correction.z;
    }
    initPhysics();
}

SoftBody::MeshData SoftBody::loadTetMesh(const std::string& filename) {
    SoftBody::MeshData meshData;
    std::ifstream file(filename);
    std::string line;
    bool readingVertices = false, readingTetrahedra = false, readingEdges = false, readingSurfaceTris = false;

    while (std::getline(file, line)) {
        if (line == "VERTICES") {
            readingVertices = true; readingTetrahedra = false; readingEdges = false; readingSurfaceTris = false;
            continue;
        }
        if (line == "TETRAHEDRA") {
            readingVertices = false; readingTetrahedra = true; readingEdges = false; readingSurfaceTris = false;
            continue;
        }
        if (line == "EDGES") {
            readingVertices = false; readingTetrahedra = false; readingEdges = true; readingSurfaceTris = false;
            continue;
        }
        if (line == "SURFACE_TRIANGLES") {
            readingVertices = false; readingTetrahedra = false; readingEdges = false; readingSurfaceTris = true;
            continue;
        }

        std::istringstream ss(line);
        if (readingVertices) {
            float x, y, z;
            ss >> x >> y >> z;
            meshData.verts.push_back(x);
            meshData.verts.push_back(y);
            meshData.verts.push_back(z);
        }
        else if (readingTetrahedra) {
            int v0, v1, v2, v3;
            ss >> v0 >> v1 >> v2 >> v3;
            meshData.tetIds.push_back(v0);
            meshData.tetIds.push_back(v1);
            meshData.tetIds.push_back(v2);
            meshData.tetIds.push_back(v3);
        }
        else if (readingEdges) {
            int e0, e1;
            ss >> e0 >> e1;
            meshData.tetEdgeIds.push_back(e0);
            meshData.tetEdgeIds.push_back(e1);
        }
        else if (readingSurfaceTris) {
            int t0, t1, t2;
            ss >> t0 >> t1 >> t2;
            meshData.tetSurfaceTriIds.push_back(t0);
            meshData.tetSurfaceTriIds.push_back(t1);
            meshData.tetSurfaceTriIds.push_back(t2);
        }
    }
    return meshData;
}


SoftBody::MeshData SoftBody::loadSurfaceMesh(const std::string& filename) {
    SoftBody::MeshData meshData;
    std::ifstream file(filename);
    std::string line;
    bool readingVertices = false, readingTetrahedra = false, readingEdges = false, readingSurfaceTris = false;

    while (std::getline(file, line)) {
        if (line == "VERTICES") {
            readingVertices = true; readingTetrahedra = false; readingEdges = false; readingSurfaceTris = false;
            continue;
        }
        if (line == "TETRAHEDRA") {
            readingVertices = false; readingTetrahedra = true; readingEdges = false; readingSurfaceTris = false;
            continue;
        }
        if (line == "EDGES") {
            readingVertices = false; readingTetrahedra = false; readingEdges = true; readingSurfaceTris = false;
            continue;
        }
        if (line == "SURFACE_TRIANGLES") {
            readingVertices = false; readingTetrahedra = false; readingEdges = false; readingSurfaceTris = true;
            continue;
        }

        std::istringstream ss(line);
        if (readingVertices) {
            float x, y, z;
            ss >> x >> y >> z;
            meshData.verts.push_back(x);
            meshData.verts.push_back(y);
            meshData.verts.push_back(z);
        }
        else if (readingSurfaceTris) {
            int t0, t1, t2;
            ss >> t0 >> t1 >> t2;
            meshData.tetSurfaceTriIds.push_back(t0);
            meshData.tetSurfaceTriIds.push_back(t1);
            meshData.tetSurfaceTriIds.push_back(t2);
        }
    }
    return meshData;
}
SoftBody::~SoftBody() {
    deleteBuffers();
}
/*
SoftBody::~SoftBody() {
    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    glDeleteBuffers(1, &EBO);
    glDeleteBuffers(1, &normalVBO);
}
*/

void SoftBody::deleteBuffers() {
    // Delete main mesh buffers
    if (VAO != 0) {
        glDeleteVertexArrays(1, &VAO);
        VAO = 0;
    }
    if (VBO != 0) {
        glDeleteBuffers(1, &VBO);
        VBO = 0;
    }
    if (EBO != 0) {
        glDeleteBuffers(1, &EBO);
        EBO = 0;
    }
    if (normalVBO != 0) {
        glDeleteBuffers(1, &normalVBO);
        normalVBO = 0;
    }

    // Delete tetrahedral mesh buffers
    if (tetVAO != 0) {
        glDeleteVertexArrays(1, &tetVAO);
        tetVAO = 0;
    }
    if (tetVBO != 0) {
        glDeleteBuffers(1, &tetVBO);
        tetVBO = 0;
    }
    if (tetEBO != 0) {
        glDeleteBuffers(1, &tetEBO);
        tetEBO = 0;
    }

    // Delete visualization mesh buffers
    for (size_t i = 0; i < visVAOs.size(); i++) {
        if (visVAOs[i] != 0) {
            glDeleteVertexArrays(1, &visVAOs[i]);
            glDeleteBuffers(1, &visVBOs[i]);
            glDeleteBuffers(1, &visEBOs[i]);
            glDeleteBuffers(1, &visNormalVBOs[i]);
        }
    }
    visVAOs.clear();
    visVBOs.clear();
    visEBOs.clear();
    visNormalVBOs.clear();
}


void SoftBody::clear() {
    // メッシュデータをクリア (const メンバーなので直接クリアできない)
    positions.clear();
    prevPositions.clear();
    velocities.clear();
    tetIds.clear();
    edgeIds.clear();
    tetSurfaceTriIds.clear();
    restVols.clear();
    edgeLengths.clear();
    invMasses.clear();

    // 表示用メッシュデータをクリア
    vis_positions.clear();
    visSurfaceTriIds.clear();
    skinningInfo.clear();

    // 一時バッファをクリア
    tempBuffer.clear();
    grads.clear();

    // アクティブな頂点情報をクリア
    activeParticles.clear();
    oldInvMasses.clear();

    // OpenGLバッファを削除
    deleteBuffers();

    // 掴み操作の状態をリセット
    grabId = -1;
    grabInvMass = 0.0f;

    // 頂点数をリセット
    numParticles = 0;
    numTets = 0;
    numVisVerts = 0;
    numVisParticles = 0;

    vis_positions_array.clear();
    visSurfaceTriIds_array.clear();
    skinningInfo_array.clear();

}


void SoftBody::initialize(const MeshData& tetMesh, const MeshData& visMesh, float edgeComp, float volComp) {
    // 既存のデータをクリア
    clear();

    // 新しいパラメータを設定
    // meshData = tetMesh; // これは許可されていない
    // vismeshData = visMesh; // これは許可されていない

    // 代わりに、個別のフィールドをコピー
    edgeCompliance = edgeComp;
    volCompliance = volComp;
    damping = 0.99f;

    std::cout << "=== SoftBody Initialization ===" << std::endl;

    // 物理シミュレーション用メッシュの初期化
    numParticles = tetMesh.verts.size() / 3;
    numTets = tetMesh.tetIds.size() / 4;

    numVisVerts = visMesh.verts.size() / 3;
    numVisParticles = visMesh.verts.size() / 3;
    vis_positions = visMesh.verts;
    visSurfaceTriIds = visMesh.tetSurfaceTriIds;

    // 物理シミュレーション用データの初期化
    positions = tetMesh.verts;
    prevPositions = tetMesh.verts;
    velocities.resize(3 * numParticles, 0.0f);

    tetIds = tetMesh.tetIds;
    tetSurfaceTriIds = tetMesh.tetSurfaceTriIds;
    edgeIds = tetMesh.tetEdgeIds;

    restVols.resize(numTets, 0.0f);
    edgeLengths.resize(edgeIds.size() / 2, 0.0f);
    invMasses.resize(numParticles, 0.0f);

    // 一時バッファの初期化
    tempBuffer.resize(4 * 3, 0.0f);
    grads.resize(4 * 3, 0.0f);

    // スキニング情報の初期化
    skinningInfo.resize(4 * numVisVerts, -1.0f);
    computeSkinningInfo(visMesh.verts);

    // 物理パラメータの初期化
    initPhysics();

    // 表示用メッシュのセットアップ
    setupVisMesh();

    // 四面体メッシュの表示設定
    setupTetMesh();

    // 法線バッファの初期化
    setupNormalBuffer();

    // 初期状態では四面体メッシュを表示
    showTetMesh = true;

    // モデル行列の初期化
    modelMatrix = glm::mat4(1.0f);

    std::cout << "=== Initialization Complete ===" << std::endl;
}



SoftBody::SoftBody(const MeshData& tetMesh,
                   const std::vector<MeshData>& visMeshes,
                   float edgeCompliance,
                   float volCompliance)
    : meshData(tetMesh)  // 物理シミュレーション用メッシュを保存
    , vismeshDataArray(visMeshes)  // 複数の表示用メッシュを保存
    , edgeCompliance(edgeCompliance)
    , volCompliance(volCompliance)
    , grabId(-1)
    , grabInvMass(0.0f)
    , damping(0.99f)
{
    // デバッグ出力の追加
    std::cout << "=== SoftBody Constructor Debug ===" << std::endl;

    // TetMeshの情報
    std::cout << "TetMesh Info:" << std::endl;
    std::cout << "  Vertices: " << tetMesh.verts.size() / 3 << std::endl;
    std::cout << "  Tetrahedra: " << tetMesh.tetIds.size() / 4 << std::endl;
    std::cout << "  Edges: " << tetMesh.tetEdgeIds.size() / 2 << std::endl;
    std::cout << "  Surface Triangles: " << tetMesh.tetSurfaceTriIds.size() / 3 << std::endl;

    // 複数のVisMeshの情報
    std::cout << "VisMesh Count: " << visMeshes.size() << std::endl;
    for (size_t i = 0; i < visMeshes.size(); i++) {
        std::cout << "VisMesh " << i << " Info:" << std::endl;
        std::cout << "  Vertices: " << visMeshes[i].verts.size() / 3 << std::endl;
        std::cout << "  Surface Triangles: " << visMeshes[i].tetSurfaceTriIds.size() / 3 << std::endl;
    }

    // 物理シミュレーション用メッシュの初期化
    std::cout << "=== 物理シミュレーション用メッシュの初期化 ===" << std::endl;
    numParticles = tetMesh.verts.size() / 3;
    numTets = tetMesh.tetIds.size() / 4;

    // 表示用メッシュ関連の配列を初期化
    vis_positions_array.resize(visMeshes.size());
    visSurfaceTriIds_array.resize(visMeshes.size());
    skinningInfo_array.resize(visMeshes.size());

    numVisVerts = 0;  // Total visualization vertices
    numVisParticles = 0;  // Total visualization particles

    // 元のメッシュデータを保存するための配列を初期化
    original_vis_positions_array.resize(visMeshes.size());

    // 各表示用メッシュの初期化
    for (size_t i = 0; i < visMeshes.size(); i++) {
        vis_positions_array[i] = visMeshes[i].verts;
        visSurfaceTriIds_array[i] = visMeshes[i].tetSurfaceTriIds;

        size_t meshVisVerts = visMeshes[i].verts.size() / 3;
        numVisVerts += meshVisVerts;
        numVisParticles += meshVisVerts;

        // スキニング情報の初期化
        skinningInfo_array[i].resize(4 * meshVisVerts, -1.0f);

        original_vis_positions_array[i] = visMeshes[i].verts;
    }

    // OpenGL関連の配列を初期化
    visVAOs.resize(visMeshes.size(), 0);
    visVBOs.resize(visMeshes.size(), 0);
    visEBOs.resize(visMeshes.size(), 0);
    visNormalVBOs.resize(visMeshes.size(), 0);

    // 物理シミュレーション用データの初期化
    std::cout << "=== 物理シミュレーション用データの初期化 ===" << std::endl;
    positions = tetMesh.verts;
    prevPositions = tetMesh.verts;
    velocities.resize(3 * numParticles, 0.0f);

    tetIds = tetMesh.tetIds;
    tetSurfaceTriIds = tetMesh.tetSurfaceTriIds;
    edgeIds = tetMesh.tetEdgeIds;

    restVols.resize(numTets, 0.0f);
    edgeLengths.resize(edgeIds.size() / 2, 0.0f);
    invMasses.resize(numParticles, 0.0f);

    // 一時バッファの初期化
    std::cout << "=== 一時バッファの初期化 ===" << std::endl;
    tempBuffer.resize(4 * 3, 0.0f);
    grads.resize(4 * 3, 0.0f);

    // コンストラクタ内
    edgeLambdas.resize(edgeIds.size() / 2, 0.0f);
    volLambdas.resize(numTets, 0.0f);

    // 各メッシュのスキニング情報を計算
    std::cout << "=== スキニング情報の計算 ===" << std::endl;
    for (size_t i = 0; i < visMeshes.size(); i++) {
        computeSkinningInfoForMesh(visMeshes[i].verts, skinningInfo_array[i]);
    }

    // 物理パラメータの初期化
    std::cout << "=== 物理パラメータの初期化 ===" << std::endl;
    initPhysics();

    // 表示用メッシュのセットアップ
    std::cout << "=== 表示用メッシュのセットアップ ===" << std::endl;
    setupVisMeshes();

    // 四面体メッシュの表示設定
    std::cout << "=== 四面体メッシュの表示設定 ===" << std::endl;
    setupTetMesh();

    // 法線バッファの初期化
    std::cout << "=== 法線バッファの初期化===" << std::endl;
    setupNormalBuffer();

    // 初期状態では四面体メッシュは表示
    std::cout << "=== 初期状態では四面体メッシュは表示===" << std::endl;
    showTetMesh = true;

    // モデル行列の初期化
    std::cout << "=== モデル行列の初期化 ===" << std::endl;
    modelMatrix = glm::mat4(1.0f);
    std::cout << "=== コンストラクタ完了 ===" << std::endl;

}


void SoftBody::setupVisMeshes() {
    // まず古いバッファを削除
    for (size_t i = 0; i < visVAOs.size(); i++) {
        if (visVAOs[i] != 0) {
            glDeleteVertexArrays(1, &visVAOs[i]);
            glDeleteBuffers(1, &visVBOs[i]);
            glDeleteBuffers(1, &visEBOs[i]);
            glDeleteBuffers(1, &visNormalVBOs[i]);
        }
    }

    // 各メッシュごとにバッファを設定
    for (size_t i = 0; i < vismeshDataArray.size(); i++) {
        glGenVertexArrays(1, &visVAOs[i]);
        glGenBuffers(1, &visVBOs[i]);
        glGenBuffers(1, &visEBOs[i]);
        glGenBuffers(1, &visNormalVBOs[i]);

        glBindVertexArray(visVAOs[i]);

        // 頂点位置
        glBindBuffer(GL_ARRAY_BUFFER, visVBOs[i]);
        glBufferData(GL_ARRAY_BUFFER, vis_positions_array[i].size() * sizeof(float),
                     vis_positions_array[i].data(), GL_DYNAMIC_DRAW);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        // 法線
        glBindBuffer(GL_ARRAY_BUFFER, visNormalVBOs[i]);
        computeVisNormalsForMesh(i);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(1);

        // インデックス
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, visEBOs[i]);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                     visSurfaceTriIds_array[i].size() * sizeof(int),
                     visSurfaceTriIds_array[i].data(), GL_STATIC_DRAW);
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}



void SoftBody::updateVisMeshes() {
    // 各メッシュの位置を更新
    for (size_t meshIdx = 0; meshIdx < vismeshDataArray.size(); meshIdx++) {
        size_t numMeshVerts = vis_positions_array[meshIdx].size() / 3;

        // スキニング情報を使って頂点位置を更新
        int nr = 0;
        for (size_t i = 0; i < numMeshVerts; i++) {
            // 四面体のインデックスを取得
            int tetNr = static_cast<int>(skinningInfo_array[meshIdx][nr++]) * 4;
            if (tetNr < 0) {
                nr += 3;  // スキニング情報をスキップ
                continue;
            }

            // バリセントリック座標（重み）を取得
            float b0 = skinningInfo_array[meshIdx][nr++];
            float b1 = skinningInfo_array[meshIdx][nr++];
            float b2 = skinningInfo_array[meshIdx][nr++];
            float b3 = 1.0f - b0 - b1 - b2;

            // 四面体の頂点インデックスを取得
            int id0 = tetIds[tetNr++];
            int id1 = tetIds[tetNr++];
            int id2 = tetIds[tetNr++];
            int id3 = tetIds[tetNr++];

            // 頂点位置を初期化
            VectorMath::vecSetZero(vis_positions_array[meshIdx], i);

            // 重み付きで四面体の各頂点の影響を加算
            VectorMath::vecAdd(vis_positions_array[meshIdx], i, positions, id0, b0);
            VectorMath::vecAdd(vis_positions_array[meshIdx], i, positions, id1, b1);
            VectorMath::vecAdd(vis_positions_array[meshIdx], i, positions, id2, b2);
            VectorMath::vecAdd(vis_positions_array[meshIdx], i, positions, id3, b3);
        }

        // OpenGLバッファを更新
        glBindBuffer(GL_ARRAY_BUFFER, visVBOs[meshIdx]);
        glBufferSubData(GL_ARRAY_BUFFER, 0,
                        vis_positions_array[meshIdx].size() * sizeof(float),
                        vis_positions_array[meshIdx].data());

        // 法線を再計算
        computeVisNormalsForMesh(meshIdx);
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);
}


void SoftBody::computeVisNormalsForMesh(size_t meshIdx) {
    size_t numMeshVerts = vis_positions_array[meshIdx].size() / 3;
    std::vector<glm::vec3> normals(numMeshVerts, glm::vec3(0.0f));

    // 面ごとに法線を計算して蓄積
    for (size_t i = 0; i < visSurfaceTriIds_array[meshIdx].size(); i += 3) {
        int id0 = visSurfaceTriIds_array[meshIdx][i];
        int id1 = visSurfaceTriIds_array[meshIdx][i + 1];
        int id2 = visSurfaceTriIds_array[meshIdx][i + 2];

        glm::vec3 p0(vis_positions_array[meshIdx][id0 * 3],
                     vis_positions_array[meshIdx][id0 * 3 + 1],
                     vis_positions_array[meshIdx][id0 * 3 + 2]);
        glm::vec3 p1(vis_positions_array[meshIdx][id1 * 3],
                     vis_positions_array[meshIdx][id1 * 3 + 1],
                     vis_positions_array[meshIdx][id1 * 3 + 2]);
        glm::vec3 p2(vis_positions_array[meshIdx][id2 * 3],
                     vis_positions_array[meshIdx][id2 * 3 + 1],
                     vis_positions_array[meshIdx][id2 * 3 + 2]);

        glm::vec3 normal = glm::normalize(glm::cross(p1 - p0, p2 - p0));

        normals[id0] += normal;
        normals[id1] += normal;
        normals[id2] += normal;
    }

    // 法線を正規化してバッファに格納
    std::vector<float> normalBuffer;
    normalBuffer.reserve(numMeshVerts * 3);

    for (const auto& n : normals) {
        glm::vec3 normalized = glm::length(n) > 0.0f ? glm::normalize(n) : n;
        normalBuffer.push_back(normalized.x);
        normalBuffer.push_back(normalized.y);
        normalBuffer.push_back(normalized.z);
    }

    // 法線バッファを更新
    glBindBuffer(GL_ARRAY_BUFFER, visNormalVBOs[meshIdx]);
    glBufferData(GL_ARRAY_BUFFER, normalBuffer.size() * sizeof(float),
                 normalBuffer.data(), GL_DYNAMIC_DRAW);
}

// 各メッシュごとにスキニング情報を計算する関数
void SoftBody::computeSkinningInfoForMesh(const std::vector<float>& visVerts,
                                          std::vector<float>& skinningInfoOut) {
    // 元のコードと同様のスキニング処理を実装
    // ただし、単一のメッシュ向けに処理する

    size_t numMeshVerts = visVerts.size() / 3;

    // 頂点用のハッシュを作成
    std::cout << "Computing skinning for mesh with " << numMeshVerts << " vertices" << std::endl;

    // バウンディングボックスの計算
    glm::vec3 tetMin(std::numeric_limits<float>::max());
    glm::vec3 tetMax(std::numeric_limits<float>::lowest());
    glm::vec3 visMin(std::numeric_limits<float>::max());
    glm::vec3 visMax(std::numeric_limits<float>::lowest());

    // 四面体メッシュのバウンディングボックス
    for (size_t i = 0; i < positions.size(); i += 3) {
        tetMin.x = std::min(tetMin.x, positions[i]);
        tetMin.y = std::min(tetMin.y, positions[i + 1]);
        tetMin.z = std::min(tetMin.z, positions[i + 2]);
        tetMax.x = std::max(tetMax.x, positions[i]);
        tetMax.y = std::max(tetMax.y, positions[i + 1]);
        tetMax.z = std::max(tetMax.z, positions[i + 2]);
    }

    // 表示用メッシュのバウンディングボックス
    for (size_t i = 0; i < visVerts.size(); i += 3) {
        visMin.x = std::min(visMin.x, visVerts[i]);
        visMin.y = std::min(visMin.y, visVerts[i + 1]);
        visMin.z = std::min(visMin.z, visVerts[i + 2]);
        visMax.x = std::max(visMax.x, visVerts[i]);
        visMax.y = std::max(visMax.y, visVerts[i + 1]);
        visMax.z = std::max(visMax.z, visVerts[i + 2]);
    }

    // スケール係数の計算
    glm::vec3 tetSize = tetMax - tetMin;
    glm::vec3 visSize = visMax - visMin;
    float maxSize = std::max({tetSize.x, tetSize.y, tetSize.z});
    float spacing = maxSize * 1.0f;

    Hash hash(spacing, numMeshVerts);
    hash.create(visVerts);

    // スキニング情報を初期化
    skinningInfoOut.assign(4 * numMeshVerts, -1.0f);
    std::vector<float> minDist(numMeshVerts, std::numeric_limits<float>::max());
    const float border = 0.05f;

    // 一時バッファ
    std::vector<float> tetCenter(3, 0.0f);
    std::vector<float> mat(9, 0.0f);
    std::vector<float> bary(4, 0.0f);

    // 各四面体について処理
    for (size_t i = 0; i < numTets; i++) {
        // 四面体の中心を計算
        std::fill(tetCenter.begin(), tetCenter.end(), 0.0f);
        for (int j = 0; j < 4; j++) {
            VectorMath::vecAdd(tetCenter, 0, positions, tetIds[4 * i + j], 0.25f);
        }

        // バウンディングスフィアの半径を計算
        float rMax = 0.0f;
        for (int j = 0; j < 4; j++) {
            float r2 = VectorMath::vecDistSquared(tetCenter, 0, positions, tetIds[4 * i + j]);
            rMax = std::max(rMax, std::sqrt(r2));
        }
        rMax += border;

        // 近傍頂点を検索
        hash.query(tetCenter, 0, rMax);
        if (hash.querySize == 0) continue;

        // 四面体の頂点インデックス
        int id0 = tetIds[4 * i];
        int id1 = tetIds[4 * i + 1];
        int id2 = tetIds[4 * i + 2];
        int id3 = tetIds[4 * i + 3];

        // 行列を計算
        VectorMath::vecSetDiff(mat, 0, positions, id0, positions, id3);
        VectorMath::vecSetDiff(mat, 1, positions, id1, positions, id3);
        VectorMath::vecSetDiff(mat, 2, positions, id2, positions, id3);
        VectorMath::matSetInverse(mat);

        // 検索された各頂点に対して処理
        for (int j = 0; j < hash.querySize; j++) {
            int id = hash.queryIds[j];

            // すでにスキニング情報がある場合はスキップ
            if (minDist[id] <= 0.0f) continue;

            // 距離チェック
            if (VectorMath::vecDistSquared(visVerts, id, tetCenter, 0) > rMax * rMax) continue;

            // 重心座標を計算
            VectorMath::vecSetDiff(bary, 0, visVerts, id, positions, id3);
            VectorMath::matSetMult(mat, bary, 0, bary, 0);
            bary[3] = 1.0f - bary[0] - bary[1] - bary[2];

            // 最大距離を計算
            float dist = 0.0f;
            for (int k = 0; k < 4; k++) {
                dist = std::max(dist, -bary[k]);
            }

            // より良い結果が見つかった場合は更新
            if (dist < minDist[id]) {
                minDist[id] = dist;
                skinningInfoOut[4 * id] = static_cast<float>(i);
                skinningInfoOut[4 * id + 1] = bary[0];
                skinningInfoOut[4 * id + 2] = bary[1];
                skinningInfoOut[4 * id + 3] = bary[2];
            }
        }
    }
}

void SoftBody::draw_Visarray(ShaderProgram& shader) {
    shader.use();

    // 現在アクティブなメッシュのみを描画
    if (activeVisMeshIndex >= 0 && activeVisMeshIndex < visVAOs.size()) {
        glBindVertexArray(visVAOs[activeVisMeshIndex]);
        glDrawElements(GL_TRIANGLES,
                       visSurfaceTriIds_array[activeVisMeshIndex].size(),
                       GL_UNSIGNED_INT, 0);
    }

    glBindVertexArray(0);
}


// 全てのメッシュを描画するメソッド（半透明表示対応）
void SoftBody::draw_AllVisMeshes(ShaderProgram& shader) {
    shader.use();

    // アルファブレンディングを有効化
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // 深度マスクの設定（半透明オブジェクトの描画時は無効にすることが一般的）
    glDepthMask(GL_FALSE);

    // 各メッシュの色を定義（アルファ値を調整して透明度を設定）
    std::vector<glm::vec4> meshColors = {
        glm::vec4(0.8f, 0.2f, 0.2f, 0.3f),  // 赤系（透明度50%）
        glm::vec4(0.9f, 0.6f, 0.6f, 0.9f),  // ピンク（透明度50%）
        glm::vec4(0.2f, 0.8f, 0.8f, 0.9f),  // 水色系（透明度50%）
        glm::vec4(0.8f, 0.2f, 0.8f, 0.9f),  // 紫系（透明度50%）
        glm::vec4(0.2f, 0.8f, 0.2f, 0.7f),  // 緑系（透明度50%）
        glm::vec4(0.2f, 0.8f, 0.2f, 0.8f),  // 緑系（透明度50%）
    };


    // 両面描画を有効にする（内側の面も表示するため）
    //glDisable(GL_CULL_FACE);

    // 優先度高く描画したいメッシュ（内部構造など）を最後に描画
    // 深度値によるソートが理想的ですが、簡易的な方法としてindexの順で描画
    for (size_t i = 0; i < visVAOs.size(); i++) {
        // メッシュごとに色を設定
        glm::vec4 vertColor = meshColors[i % meshColors.size()];
        shader.setUniform("vertColor", vertColor);

        glBindVertexArray(visVAOs[i]);
        glDrawElements(GL_TRIANGLES,
                       visSurfaceTriIds_array[i].size(),
                       GL_UNSIGNED_INT, 0);
    }

    // 元の設定に戻す
    glEnable(GL_CULL_FACE);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);

    glBindVertexArray(0);
}

// アクティブなビジュアルメッシュを設定するメソッド
void SoftBody::setActiveVisMesh(int index) {
    if (index >= 0 && index < static_cast<int>(vismeshDataArray.size())) {
        activeVisMeshIndex = index;
    }
}

// 三角形情報を保持する構造体
struct TriangleInfo {
    size_t index;    // 三角形のインデックス
    float distance;  // カメラからの距離
};

void SoftBody::draw_AllVisMeshes(ShaderProgram& shader, glm::vec3 camPos) {
    shader.use();

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);

    // カメラ位置を取得
    glm::vec3 cameraPos = camPos;

    // 各メッシュの色を定義（アルファ値を調整して透明度を設定）
    std::vector<glm::vec4> meshColors = {
        glm::vec4(0.8f, 0.2f, 0.2f, 1.0f),  // 赤系（透明度50%）
        glm::vec4(0.9f, 0.6f, 0.6f, 0.9f),  // ピンク（透明度50%）
        glm::vec4(0.2f, 0.8f, 0.8f, 0.9f),  // 水色系（透明度50%）
        glm::vec4(0.8f, 0.2f, 0.8f, 0.9f),  // 紫系（透明度50%）
        glm::vec4(0.2f, 0.8f, 0.2f, 0.0f),  // 緑系（透明度50%）
        glm::vec4(0.2f, 0.8f, 0.2f, 0.8f),  // 緑系（透明度50%）
    };

    // すべてのメッシュから全ての三角形を抽出して一緒にソート
    struct GlobalTriangleInfo {
        size_t meshIndex;       // どのメッシュに属するか
        size_t triangleIndex;   // メッシュ内の三角形インデックス
        float distance;         // カメラからの距離
    };

    std::vector<GlobalTriangleInfo> allTriangles;

    // 全てのメッシュから三角形情報を収集
    for (size_t meshIdx = 0; meshIdx < visVAOs.size(); meshIdx++) {
        const auto& positions = vis_positions_array[meshIdx];
        const auto& indices = visSurfaceTriIds_array[meshIdx];

        for (size_t j = 0; j < indices.size(); j += 3) {
            int idx1 = indices[j];
            int idx2 = indices[j + 1];
            int idx3 = indices[j + 2];

            glm::vec3 v1(positions[idx1 * 3], positions[idx1 * 3 + 1], positions[idx1 * 3 + 2]);
            glm::vec3 v2(positions[idx2 * 3], positions[idx2 * 3 + 1], positions[idx2 * 3 + 2]);
            glm::vec3 v3(positions[idx3 * 3], positions[idx3 * 3 + 1], positions[idx3 * 3 + 2]);

            glm::vec3 center = (v1 + v2 + v3) / 3.0f;
            float distance = glm::length(cameraPos - center);

            allTriangles.push_back({meshIdx, j / 3, distance});
        }
    }

    // カメラから遠い順に全三角形をソート
    std::sort(allTriangles.begin(), allTriangles.end(),
              [](const GlobalTriangleInfo& a, const GlobalTriangleInfo& b) {
                  return a.distance > b.distance;
              });

    // ソートされた順序ですべての三角形を描画
    GLuint lastVAO = -1;        // VAOの切り替えを最小限にするための前回値
    glm::vec4 lastColor;        // 色の切り替えを最小限にするための前回値
    bool colorInitialized = false;

    for (const auto& tri : allTriangles) {
        size_t meshIdx = tri.meshIndex;
        size_t triIdx = tri.triangleIndex;

        // VAOの切り替えが必要な場合のみ切り替え
        if (lastVAO != visVAOs[meshIdx]) {
            glBindVertexArray(visVAOs[meshIdx]);
            lastVAO = visVAOs[meshIdx];
        }

        // 色の切り替えが必要な場合のみ切り替え
        glm::vec4 currentColor = meshColors[meshIdx % meshColors.size()];
        if (!colorInitialized || lastColor != currentColor) {
            shader.setUniform("vertColor", currentColor);
            lastColor = currentColor;
            colorInitialized = true;
        }

        // 三角形を描画
        glDrawElements(GL_TRIANGLES, 3, GL_UNSIGNED_INT,
                       (void*)(triIdx * 3 * sizeof(GLuint)));
    }

    // 元の設定に戻す
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glBindVertexArray(0);
}



/*
void SoftBody::draw_AllVisMeshes(ShaderProgram& shader,glm::vec3 camPos) {
    shader.use();

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //・glBlendFuncSeparate(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA, GL_ONE, GL_ONE);
    //・glBlendFunc(GL_SRC_ALPHA, GL_ONE);
    glDepthMask(GL_FALSE);


    // カメラ位置を取得（シェーダーから取得するか、外部から渡す）
    glm::vec3 cameraPos = camPos;

    // 各メッシュの色を定義（アルファ値を調整して透明度を設定）
    std::vector<glm::vec4> meshColors = {
        glm::vec4(0.8f, 0.2f, 0.2f, 0.5f),  // 赤系（透明度70%）
        glm::vec4(0.2f, 0.2f, 0.8f, 0.5f),  // 青系（透明度70%）
        glm::vec4(0.2f, 0.8f, 0.8f, 0.5f),   // 水色系（透明度70%）
        glm::vec4(0.8f, 0.2f, 0.8f, 0.5f),  // 紫系（透明度70%）
        glm::vec4(0.2f, 0.8f, 0.2f, 0.5f),  // 緑系（透明度70%）
        glm::vec4(0.2f, 0.2f, 0.8f, 0.f),  // 青系（透明度70%）
        glm::vec4(0.8f, 0.8f, 0.2f, 0.5f),  // 黄系（透明度70%）
        glm::vec4(0.8f, 0.2f, 0.8f, 0.5f),  // 紫系（透明度70%）
        glm::vec4(0.2f, 0.8f, 0.8f, 0.5f)   // 水色系（透明度70%）
    };

    // メッシュごとに三角形を深度ソート
    for (size_t i = 0; i < visVAOs.size(); i++) {
        glm::vec4 vertColor = meshColors[i % meshColors.size()];
        shader.setUniform("vertColor", vertColor);

        // 三角形ごとの中心点を計算して距離でソート
        std::vector<TriangleInfo> sortedTriangles;
        const auto& positions = vis_positions_array[i];
        const auto& indices = visSurfaceTriIds_array[i];

        for (size_t j = 0; j < indices.size(); j += 3) {
            int idx1 = indices[j];
            int idx2 = indices[j + 1];
            int idx3 = indices[j + 2];

            glm::vec3 v1(positions[idx1 * 3], positions[idx1 * 3 + 1], positions[idx1 * 3 + 2]);
            glm::vec3 v2(positions[idx2 * 3], positions[idx2 * 3 + 1], positions[idx2 * 3 + 2]);
            glm::vec3 v3(positions[idx3 * 3], positions[idx3 * 3 + 1], positions[idx3 * 3 + 2]);

            glm::vec3 center = (v1 + v2 + v3) / 3.0f;
            float distance = glm::length(cameraPos - center);

            sortedTriangles.push_back({j / 3, distance});
        }

        // カメラから遠い順にソート
        std::sort(sortedTriangles.begin(), sortedTriangles.end(),
                 [](const TriangleInfo& a, const TriangleInfo& b) {
                     return a.distance > b.distance;
                 });

        // ソートされた順序で三角形を描画
        glBindVertexArray(visVAOs[i]);
        for (const auto& tri : sortedTriangles) {
            size_t triIdx = tri.index;
            glDrawElements(GL_TRIANGLES, 3, GL_UNSIGNED_INT,
                          (void*)(triIdx * 3 * sizeof(GLuint)));
        }
    }

    // 元の設定に戻す
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    glBindVertexArray(0);
}
*/

/*
void SoftBody::computeSkinningInfo(const std::vector<float>& visVerts) {
    // サイズチェック
    if (visVerts.empty() || positions.empty() || tetIds.empty()) {
        std::cout << "Error: Invalid mesh data" << std::endl;
        return;
    }

    size_t numVisVerts = visVerts.size() / 3;
    std::cout << "Number of visual vertices: " << numVisVerts << std::endl;

    // バウンディングボックスの計算
    glm::vec3 tetMin(std::numeric_limits<float>::max());
    glm::vec3 tetMax(std::numeric_limits<float>::lowest());
    glm::vec3 visMin(std::numeric_limits<float>::max());
    glm::vec3 visMax(std::numeric_limits<float>::lowest());

    // 四面体メッシュのバウンディングボックス
    for (size_t i = 0; i < positions.size(); i += 3) {
        tetMin.x = std::min(tetMin.x, positions[i]);
        tetMin.y = std::min(tetMin.y, positions[i + 1]);
        tetMin.z = std::min(tetMin.z, positions[i + 2]);
        tetMax.x = std::max(tetMax.x, positions[i]);
        tetMax.y = std::max(tetMax.y, positions[i + 1]);
        tetMax.z = std::max(tetMax.z, positions[i + 2]);
    }

    // 表示用メッシュのバウンディングボックス
    for (size_t i = 0; i < visVerts.size(); i += 3) {
        visMin.x = std::min(visMin.x, visVerts[i]);
        visMin.y = std::min(visMin.y, visVerts[i + 1]);
        visMin.z = std::min(visMin.z, visVerts[i + 2]);
        visMax.x = std::max(visMax.x, visVerts[i]);
        visMax.y = std::max(visMax.y, visVerts[i + 1]);
        visMax.z = std::max(visMax.z, visVerts[i + 2]);
    }

    // スケール係数の計算
    glm::vec3 tetSize = tetMax - tetMin;
    glm::vec3 visSize = visMax - visMin;
    float maxSize = std::max({tetSize.x, tetSize.y, tetSize.z});
    float spacing = maxSize * 0.1f;  // メッシュサイズに応じたスペーシング
    std::cout << "rMax = std::min(rMax + spacing, maxSize * 0.5f);" << std::endl;
    std::cout << "spacing;" << spacing << std::endl;
    std::cout << "maxSize;" << maxSize << std::endl;
    std::cout << "Hash hash(spacing, numVisVerts);" << std::endl;
    // ハッシュの初期化
    Hash hash(spacing, numVisVerts);
    hash.create(visVerts);

    // スキニング情報を初期化
    skinningInfo.assign(4 * numVisVerts, -1.0f);
    std::vector<float> minDist(numVisVerts, std::numeric_limits<float>::max());
    std::vector<bool> processed(numVisVerts, false);
    int remainingVertices = numVisVerts;

    // 一時バッファ
    std::vector<float> tetCenter(3, 0.0f);
    std::vector<float> mat(9, 0.0f);
    std::vector<float> bary(4, 0.0f);

    // 各四面体について処理
    for (size_t i = 0; i < tetIds.size() / 4 && remainingVertices > 0; i++) {
        // 四面体の頂点インデックスの範囲チェック
        if (4 * i + 3 >= tetIds.size()) {
            std::cout << "Error: Tetrahedron index out of range" << std::endl;
            break;
        }

        // 四面体の頂点インデックス
        int id0 = tetIds[4 * i];
        int id1 = tetIds[4 * i + 1];
        int id2 = tetIds[4 * i + 2];
        int id3 = tetIds[4 * i + 3];

        // インデックスの範囲チェック
        if (id0 >= positions.size() / 3 || id1 >=  positions.size() / 3 ||
            id2 >=  positions.size() / 3 || id3 >=  positions.size() / 3) {
            std::cout << "Error: Vertex index out of range for tetrahedron " << i << std::endl;
            continue;
        }

        // 四面体の中心を計算
        std::fill(tetCenter.begin(), tetCenter.end(), 0.0f);
        for (int j = 0; j < 4; j++) {
            VectorMath::vecAdd(tetCenter, 0,  positions, tetIds[4 * i + j], 0.25f);
        }

        // バウンディングスフィアの半径を計算
        float rMax = 0.0f;
        for (int j = 0; j < 4; j++) {
            float r2 = VectorMath::vecDistSquared(tetCenter, 0,  positions, tetIds[4 * i + j]);
            rMax = std::max(rMax, std::sqrt(r2));
        }
        //rMax = std::min(rMax + spacing, maxSize * 0.5f);  // 検索半径を制限
        const float border = 0.05f;
        rMax += border;

        // 近傍頂点を検索
        hash.query(tetCenter, 0, rMax);
        if (hash.querySize == 0) continue;

        // 行列を計算
        VectorMath::vecSetDiff(mat, 0,  positions, id0,  positions, id3);
        VectorMath::vecSetDiff(mat, 1,  positions, id1,  positions, id3);
        VectorMath::vecSetDiff(mat, 2,  positions, id2,  positions, id3);
        VectorMath::matSetInverse(mat);

        // 検索された各頂点に対して処理
        for (int j = 0; j < hash.querySize; j++) {
            int id = hash.queryIds[j];

            // 範囲チェック
            if (id < 0 || id >= numVisVerts) {
                continue;
            }

            // すでに処理済みの頂点をスキップ
            if (processed[id]) continue;

            // 距離チェック
            float currentDist = VectorMath::vecDistSquared(visVerts, id, tetCenter, 0);
            if (currentDist > rMax * rMax) continue;

            // 重心座標を計算
            VectorMath::vecSetDiff(bary, 0, visVerts, id,  positions, id3);
            VectorMath::matSetMult(mat, bary, 0, bary, 0);
            bary[3] = 1.0f - bary[0] - bary[1] - bary[2];

            // 最大距離を計算
            float dist = 0.0f;
            for (int k = 0; k < 4; k++) {
                dist = std::max(dist, -bary[k]);
            }

            // より良い結果が見つかった場合は更新
            if (dist < minDist[id]) {
                minDist[id] = dist;
                skinningInfo[4 * id] = static_cast<float>(i);
                skinningInfo[4 * id + 1] = bary[0];
                skinningInfo[4 * id + 2] = bary[1];
                skinningInfo[4 * id + 3] = bary[2];

                if (!processed[id]) {
                    processed[id] = true;
                    remainingVertices--;
                }
            }
        }

        // 進捗報告
        if (i % 100 == 0) {
            std::cout << "Processing tetrahedron " << i << "/" << (tetIds.size() / 4)
                      << ", Remaining vertices: " << remainingVertices << std::endl;
        }
    }

    // 処理結果の報告
    std::cout << "Skinning computation completed:" << std::endl;
    std::cout << "Total vertices: " << numVisVerts << std::endl;
    std::cout << "Processed vertices: " << (numVisVerts - remainingVertices) << std::endl;
    std::cout << "Remaining unprocessed: " << remainingVertices << std::endl;

    // 未処理の頂点がある場合の警告
    if (remainingVertices > 0) {
        std::cout << "Warning: Some vertices remained unprocessed!" << std::endl;
    }
}
*/

/*
void SoftBody::updateVisMeshes(int correctionMeshIdx = -1) {
    // 各メッシュの位置を更新
    for (size_t meshIdx = 0; meshIdx < vismeshDataArray.size(); meshIdx++) {
        size_t numMeshVerts = vis_positions_array[meshIdx].size() / 3;

        // スキニング情報を使って頂点位置を更新
        int nr = 0;
        for (size_t i = 0; i < numMeshVerts; i++) {
            // 四面体のインデックスを取得
            int tetNr = static_cast<int>(skinningInfo_array[meshIdx][nr++]) * 4;
            if (tetNr < 0) {
                nr += 3;  // スキニング情報をスキップ
                continue;
            }
            // バリセントリック座標（重み）を取得
            float b0 = skinningInfo_array[meshIdx][nr++];
            float b1 = skinningInfo_array[meshIdx][nr++];
            float b2 = skinningInfo_array[meshIdx][nr++];
            float b3 = 1.0f - b0 - b1 - b2;
            // 四面体の頂点インデックスを取得
            int id0 = tetIds[tetNr++];
            int id1 = tetIds[tetNr++];
            int id2 = tetIds[tetNr++];
            int id3 = tetIds[tetNr++];
            // 頂点位置を初期化
            VectorMath::vecSetZero(vis_positions_array[meshIdx], i);
            // 重み付きで四面体の各頂点の影響を加算
            VectorMath::vecAdd(vis_positions_array[meshIdx], i, positions, id0, b0);
            VectorMath::vecAdd(vis_positions_array[meshIdx], i, positions, id1, b1);
            VectorMath::vecAdd(vis_positions_array[meshIdx], i, positions, id2, b2);
            VectorMath::vecAdd(vis_positions_array[meshIdx], i, positions, id3, b3);
        }

        // 指定されたメッシュのみ補正処理を実行
        bool applyCorrection = (correctionMeshIdx < 0 || static_cast<int>(meshIdx) == correctionMeshIdx);

        if (applyCorrection) {
            // 現在の位置を一時的に保存（変形前）
            std::vector<float> pre_update_positions = vis_positions_array[meshIdx];

            // 初回更新時の処理（pre_update_positionsが初期化されていない場合）
            bool isFirstUpdate = pre_update_positions.size() < numMeshVerts * 3;

            // このメッシュの異常頂点を検出
            std::vector<bool> isAbnormal(numMeshVerts, false);

            // 各頂点の変形量を確認
            for (size_t i = 0; i < numMeshVerts; i++) {
                // スキニング情報がない場合は問題あり
                if (4 * i < skinningInfo_array[meshIdx].size() && skinningInfo_array[meshIdx][4 * i] < 0.0f) {
                    isAbnormal[i] = true;
                    continue;
                }

                // 急激な変形を検出（初回更新時はスキップ）
                if (!isFirstUpdate && 3 * i + 2 < pre_update_positions.size()) {
                    glm::vec3 prePos(
                        pre_update_positions[3 * i],
                        pre_update_positions[3 * i + 1],
                        pre_update_positions[3 * i + 2]
                        );

                    glm::vec3 currentPos(
                        vis_positions_array[meshIdx][3 * i],
                        vis_positions_array[meshIdx][3 * i + 1],
                        vis_positions_array[meshIdx][3 * i + 2]
                        );

                    // 変形量が閾値を超えた場合、異常とマーク
                    if (glm::distance(prePos, currentPos) > maxDeformationThreshold) {
                        isAbnormal[i] = true;
                    }
                }
            }

            // 隣接頂点リストを構築（このメッシュ用）
            std::vector<std::vector<int>> adjacencyList;
            buildAdjacencyListForMesh(meshIdx, adjacencyList);

            // 異常頂点の伝播
            std::vector<bool> expandedAbnormal = isAbnormal;
            for (size_t i = 0; i < numMeshVerts; i++) {
                if (isAbnormal[i]) {
                    // 異常頂点の隣接頂点をチェック
                    for (int neighborIdx : adjacencyList[i]) {
                        if (neighborIdx >= 0 && neighborIdx < numMeshVerts) {
                            // 隣接する異常頂点の数をカウント
                            int abnormalNeighbors = 0;
                            for (int secondNeighborIdx : adjacencyList[neighborIdx]) {
                                if (secondNeighborIdx >= 0 && secondNeighborIdx < numMeshVerts &&
                                    isAbnormal[secondNeighborIdx]) {
                                    abnormalNeighbors++;
                                }
                            }

                            // 複数の異常頂点に隣接している場合、この頂点も異常とマーク
                            if (abnormalNeighbors > 1) {
                                expandedAbnormal[neighborIdx] = true;
                            }
                        }
                    }
                }
            }

            // 異常頂点の補正
            correctAbnormalVerticesForMesh(meshIdx, expandedAbnormal, adjacencyList);
        }

        // OpenGLバッファを更新
        glBindBuffer(GL_ARRAY_BUFFER, visVBOs[meshIdx]);
        glBufferSubData(GL_ARRAY_BUFFER, 0,
                        vis_positions_array[meshIdx].size() * sizeof(float),
                        vis_positions_array[meshIdx].data());

        // 法線を再計算
        computeVisNormalsForMesh(meshIdx);
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);
}
*/


void SoftBody::updateVisMeshes(int correctionMeshIdx = -1) {
    // 各メッシュの位置を更新
    for (size_t meshIdx = 0; meshIdx < vismeshDataArray.size(); meshIdx++) {
        size_t numMeshVerts = vis_positions_array[meshIdx].size() / 3;

        // スキニング情報を使って頂点位置を更新
        int nr = 0;
        for (size_t i = 0; i < numMeshVerts; i++) {
            // 四面体のインデックスを取得
            int tetNr = static_cast<int>(skinningInfo_array[meshIdx][nr++]) * 4;
            if (tetNr < 0) {
                nr += 3;  // スキニング情報をスキップ
                continue;
            }
            // バリセントリック座標（重み）を取得
            float b0 = skinningInfo_array[meshIdx][nr++];
            float b1 = skinningInfo_array[meshIdx][nr++];
            float b2 = skinningInfo_array[meshIdx][nr++];
            float b3 = 1.0f - b0 - b1 - b2;
            // 四面体の頂点インデックスを取得
            int id0 = tetIds[tetNr++];
            int id1 = tetIds[tetNr++];
            int id2 = tetIds[tetNr++];
            int id3 = tetIds[tetNr++];
            // 頂点位置を初期化
            VectorMath::vecSetZero(vis_positions_array[meshIdx], i);
            // 重み付きで四面体の各頂点の影響を加算
            VectorMath::vecAdd(vis_positions_array[meshIdx], i, positions, id0, b0);
            VectorMath::vecAdd(vis_positions_array[meshIdx], i, positions, id1, b1);
            VectorMath::vecAdd(vis_positions_array[meshIdx], i, positions, id2, b2);
            VectorMath::vecAdd(vis_positions_array[meshIdx], i, positions, id3, b3);
        }

        // 指定されたメッシュのみ補正処理を実行
        bool applyCorrection = (correctionMeshIdx < 0 || static_cast<int>(meshIdx) == correctionMeshIdx);

        if (applyCorrection) {
            // スキニング情報がない頂点をカウント（最適化のため）
            int noSkinningCount = 0;
            for (size_t i = 0; i < numMeshVerts; i++) {
                if (4 * i < skinningInfo_array[meshIdx].size() && skinningInfo_array[meshIdx][4 * i] < 0.0f) {
                    noSkinningCount++;
                }
            }

            // スキニング情報がない頂点が少ない場合（5%未満）は
            // 軽量版の検出と補正を実行
            if (noSkinningCount < numMeshVerts * 0.05) {
                // 問題のある頂点のみを処理
                std::vector<bool> isAbnormal(numMeshVerts, false);

                // スキニング情報がない頂点のみを異常として検出（高速）
                for (size_t i = 0; i < numMeshVerts; i++) {
                    if (4 * i < skinningInfo_array[meshIdx].size() && skinningInfo_array[meshIdx][4 * i] < 0.0f) {
                        isAbnormal[i] = true;
                    }
                }

                // 隣接リストを取得（キャッシュがあれば再利用）
                if (cachedAdjacencyLists.size() <= meshIdx || cachedAdjacencyLists[meshIdx].empty()) {
                    if (cachedAdjacencyLists.size() <= meshIdx) {
                        cachedAdjacencyLists.resize(meshIdx + 1);
                    }
                    buildAdjacencyListForMesh(meshIdx, cachedAdjacencyLists[meshIdx]);
                }

                // 簡易補正を適用（異常頂点のみ）
                std::vector<float> corrected_positions = vis_positions_array[meshIdx];
                for (size_t i = 0; i < numMeshVerts; i++) {
                    if (isAbnormal[i]) {
                        applySmoothingForMesh(meshIdx, i, corrected_positions, cachedAdjacencyLists[meshIdx]);
                    }
                }

                // 補正を適用
                for (size_t i = 0; i < numMeshVerts; i++) {
                    if (isAbnormal[i]) {
                        float blendFactor = 0.8f; // 強めの補正

                        vis_positions_array[meshIdx][3 * i] = (1.0f - blendFactor) * vis_positions_array[meshIdx][3 * i] +
                                                              blendFactor * corrected_positions[3 * i];
                        vis_positions_array[meshIdx][3 * i + 1] = (1.0f - blendFactor) * vis_positions_array[meshIdx][3 * i + 1] +
                                                                  blendFactor * corrected_positions[3 * i + 1];
                        vis_positions_array[meshIdx][3 * i + 2] = (1.0f - blendFactor) * vis_positions_array[meshIdx][3 * i + 2] +
                                                                  blendFactor * corrected_positions[3 * i + 2];
                    }
                }
            } else {
                // 問題が多い場合は通常の詳細検出と補正を実行
                std::vector<float> pre_update_positions = vis_positions_array[meshIdx];
                bool isFirstUpdate = pre_update_positions.size() < numMeshVerts * 3;
                std::vector<bool> isAbnormal(numMeshVerts, false);

                // 各頂点の変形量を確認
                for (size_t i = 0; i < numMeshVerts; i++) {
                    // スキニング情報がない場合は問題あり
                    if (4 * i < skinningInfo_array[meshIdx].size() && skinningInfo_array[meshIdx][4 * i] < 0.0f) {
                        isAbnormal[i] = true;
                        continue;
                    }

                    // 急激な変形を検出（初回更新時はスキップ）
                    if (!isFirstUpdate && 3 * i + 2 < pre_update_positions.size()) {
                        glm::vec3 prePos(
                            pre_update_positions[3 * i],
                            pre_update_positions[3 * i + 1],
                            pre_update_positions[3 * i + 2]
                            );

                        glm::vec3 currentPos(
                            vis_positions_array[meshIdx][3 * i],
                            vis_positions_array[meshIdx][3 * i + 1],
                            vis_positions_array[meshIdx][3 * i + 2]
                            );

                        // 変形量が閾値を超えた場合、異常とマーク
                        if (glm::distance(prePos, currentPos) > maxDeformationThreshold) {
                            isAbnormal[i] = true;
                        }
                    }
                }

                // 隣接リストを取得（キャッシュがあれば再利用）
                if (cachedAdjacencyLists.size() <= meshIdx || cachedAdjacencyLists[meshIdx].empty()) {
                    if (cachedAdjacencyLists.size() <= meshIdx) {
                        cachedAdjacencyLists.resize(meshIdx + 1);
                    }
                    buildAdjacencyListForMesh(meshIdx, cachedAdjacencyLists[meshIdx]);
                }

                // 異常頂点の伝播
                std::vector<bool> expandedAbnormal = isAbnormal;
                for (size_t i = 0; i < numMeshVerts; i++) {
                    if (isAbnormal[i]) {
                        // 異常頂点の隣接頂点をチェック
                        for (int neighborIdx : cachedAdjacencyLists[meshIdx][i]) {
                            if (neighborIdx >= 0 && neighborIdx < numMeshVerts) {
                                // 隣接する異常頂点の数をカウント
                                int abnormalNeighbors = 0;
                                for (int secondNeighborIdx : cachedAdjacencyLists[meshIdx][neighborIdx]) {
                                    if (secondNeighborIdx >= 0 && secondNeighborIdx < numMeshVerts &&
                                        isAbnormal[secondNeighborIdx]) {
                                        abnormalNeighbors++;
                                    }
                                }

                                // 複数の異常頂点に隣接している場合、この頂点も異常とマーク
                                if (abnormalNeighbors > 1) {
                                    expandedAbnormal[neighborIdx] = true;
                                }
                            }
                        }
                    }
                }

                // 異常頂点の補正
                correctAbnormalVerticesForMesh(meshIdx, expandedAbnormal, cachedAdjacencyLists[meshIdx]);
            }
        }

        // OpenGLバッファを更新
        glBindBuffer(GL_ARRAY_BUFFER, visVBOs[meshIdx]);
        glBufferSubData(GL_ARRAY_BUFFER, 0,
                        vis_positions_array[meshIdx].size() * sizeof(float),
                        vis_positions_array[meshIdx].data());

        // 法線を再計算
        computeVisNormalsForMesh(meshIdx);
    }

    glBindBuffer(GL_ARRAY_BUFFER, 0);
}

// メッシュごとの隣接リスト構築
void SoftBody::buildAdjacencyListForMesh(size_t meshIdx, std::vector<std::vector<int>>& adjacencyList) {
    size_t numMeshVerts = vis_positions_array[meshIdx].size() / 3;
    adjacencyList.resize(numMeshVerts);

    // 三角形ポリゴンから隣接関係を構築
    for (size_t i = 0; i < visSurfaceTriIds_array[meshIdx].size(); i += 3) {
        int id0 = visSurfaceTriIds_array[meshIdx][i];
        int id1 = visSurfaceTriIds_array[meshIdx][i + 1];
        int id2 = visSurfaceTriIds_array[meshIdx][i + 2];

        // インデックスの範囲チェック
        if (id0 >= 0 && id0 < numMeshVerts &&
            id1 >= 0 && id1 < numMeshVerts &&
            id2 >= 0 && id2 < numMeshVerts) {
            adjacencyList[id0].push_back(id1);
            adjacencyList[id0].push_back(id2);
            adjacencyList[id1].push_back(id0);
            adjacencyList[id1].push_back(id2);
            adjacencyList[id2].push_back(id0);
            adjacencyList[id2].push_back(id1);
        }
    }

    // 重複を削除
    for (auto& neighbors : adjacencyList) {
        std::sort(neighbors.begin(), neighbors.end());
        neighbors.erase(std::unique(neighbors.begin(), neighbors.end()), neighbors.end());
    }
}

// メッシュごとの異常頂点補正
void SoftBody::correctAbnormalVerticesForMesh(size_t meshIdx, const std::vector<bool>& isAbnormal,
                                              const std::vector<std::vector<int>>& adjacencyList) {
    // 補正が必要な頂点があるか確認
    bool needsCorrection = false;
    for (bool abnormal : isAbnormal) {
        if (abnormal) {
            needsCorrection = true;
            break;
        }
    }

    if (!needsCorrection) {
        return;
    }

    size_t numMeshVerts = vis_positions_array[meshIdx].size() / 3;

    // 補正された頂点位置を格納
    std::vector<float> corrected_positions = vis_positions_array[meshIdx];

    // 異常と判定された頂点を補正
    for (size_t i = 0; i < numMeshVerts; i++) {
        if (isAbnormal[i]) {
            applySmoothingForMesh(meshIdx, i, corrected_positions, adjacencyList);
        }
    }

    // 補正結果を適用（必要に応じてブレンド係数を調整）
    for (size_t i = 0; i < numMeshVerts; i++) {
        if (isAbnormal[i]) {
            // ブレンド係数の計算
            float blendFactor = 0.5f; // 調整可能なパラメータ

            // スキニング情報がない場合は補正を強くする
            if (4 * i < skinningInfo_array[meshIdx].size() && skinningInfo_array[meshIdx][4 * i] < 0.0f) {
                blendFactor = 0.8f;
            }

            // 周囲の異常頂点の数に応じて補正強度を調整
            int abnormalNeighbors = 0;
            for (int neighborIdx : adjacencyList[i]) {
                if (neighborIdx >= 0 && neighborIdx < isAbnormal.size() && isAbnormal[neighborIdx]) {
                    abnormalNeighbors++;
                }
            }

            // 異常な隣接頂点が多い場合、より強く補正
            if (abnormalNeighbors > 2) {
                blendFactor = std::min(blendFactor + 0.2f, 0.9f);
            }

            // 補正位置とオリジナル位置をブレンド
            vis_positions_array[meshIdx][3 * i] = (1.0f - blendFactor) * vis_positions_array[meshIdx][3 * i] +
                                                  blendFactor * corrected_positions[3 * i];
            vis_positions_array[meshIdx][3 * i + 1] = (1.0f - blendFactor) * vis_positions_array[meshIdx][3 * i + 1] +
                                                      blendFactor * corrected_positions[3 * i + 1];
            vis_positions_array[meshIdx][3 * i + 2] = (1.0f - blendFactor) * vis_positions_array[meshIdx][3 * i + 2] +
                                                      blendFactor * corrected_positions[3 * i + 2];
        }
    }
}

// メッシュごとのスムージング適用
void SoftBody::applySmoothingForMesh(size_t meshIdx, int vertexIdx,
                                     std::vector<float>& corrected_positions,
                                     const std::vector<std::vector<int>>& adjacencyList) {
    size_t numMeshVerts = vis_positions_array[meshIdx].size() / 3;

    if (vertexIdx < 0 || vertexIdx >= numMeshVerts || adjacencyList[vertexIdx].empty()) {
        return;
    }

    const std::vector<int>& neighbors = adjacencyList[vertexIdx];

    // 正常な隣接頂点を使って位置を推定
    glm::vec3 avgPosition(0.0f);
    float totalWeight = 0.0f;
    int validNeighbors = 0;

    for (int neighborIdx : neighbors) {
        if (neighborIdx < 0 || neighborIdx >= numMeshVerts) continue;

        // スキニング情報がある（正常な）隣接頂点を優先
        bool hasValidSkinning = 4 * neighborIdx < skinningInfo_array[meshIdx].size() &&
                                skinningInfo_array[meshIdx][4 * neighborIdx] >= 0.0f;

        glm::vec3 neighborPos(
            vis_positions_array[meshIdx][3 * neighborIdx],
            vis_positions_array[meshIdx][3 * neighborIdx + 1],
            vis_positions_array[meshIdx][3 * neighborIdx + 2]
            );

        // 元のメッシュでの距離に基づく重み付け
        float weight = 1.0f;
        if (!original_vis_positions_array.empty() &&
            original_vis_positions_array[meshIdx].size() >= 3 * numMeshVerts) {

            glm::vec3 origPos1(
                original_vis_positions_array[meshIdx][3 * vertexIdx],
                original_vis_positions_array[meshIdx][3 * vertexIdx + 1],
                original_vis_positions_array[meshIdx][3 * vertexIdx + 2]
                );

            glm::vec3 origPos2(
                original_vis_positions_array[meshIdx][3 * neighborIdx],
                original_vis_positions_array[meshIdx][3 * neighborIdx + 1],
                original_vis_positions_array[meshIdx][3 * neighborIdx + 2]
                );

            float origDist = glm::distance(origPos1, origPos2);
            weight = 1.0f / (origDist * origDist + 0.0001f);
        } else {
            // 元の距離が使えない場合は現在の距離を使用
            glm::vec3 currentPos(
                vis_positions_array[meshIdx][3 * vertexIdx],
                vis_positions_array[meshIdx][3 * vertexIdx + 1],
                vis_positions_array[meshIdx][3 * vertexIdx + 2]
                );

            float dist = glm::distance(currentPos, neighborPos);
            weight = 1.0f / (dist * dist + 0.0001f);
        }

        // 正常な頂点の重みを高くする
        if (hasValidSkinning) {
            weight *= 2.0f;
            validNeighbors++;
        }

        avgPosition += weight * neighborPos;
        totalWeight += weight;
    }

    // 有効な隣接点が十分あるか確認
    const int minValidNeighbors = 1;
    bool useAllNeighbors = validNeighbors < minValidNeighbors;

    // 有効な隣接点が少なすぎる場合は再計算
    if (useAllNeighbors && !neighbors.empty()) {
        avgPosition = glm::vec3(0.0f);
        totalWeight = 0.0f;

        for (int neighborIdx : neighbors) {
            if (neighborIdx < 0 || neighborIdx >= numMeshVerts) continue;

            glm::vec3 neighborPos(
                vis_positions_array[meshIdx][3 * neighborIdx],
                vis_positions_array[meshIdx][3 * neighborIdx + 1],
                vis_positions_array[meshIdx][3 * neighborIdx + 2]
                );

            // 単純な平均を使用
            avgPosition += neighborPos;
            totalWeight += 1.0f;
        }
    }

    if (totalWeight > 0.0f) {
        avgPosition /= totalWeight;

        // 補正位置を設定
        corrected_positions[3 * vertexIdx] = avgPosition.x;
        corrected_positions[3 * vertexIdx + 1] = avgPosition.y;
        corrected_positions[3 * vertexIdx + 2] = avgPosition.z;
    }
}
