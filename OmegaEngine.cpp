#pragma once
#define dDOUBLE
#include <ode/ode.h>
#include <vector>
#include <array>
#include <cmath>
#include <algorithm>
#include "Materials.cpp"

constexpr float ARENA_HALF_SIZE = 750.0f;
constexpr float PLAYER_EYE_HEIGHT = 1.75f;
constexpr float PLAYER_RADIUS = 0.42f;

struct RigidBody {
    dBodyID body = nullptr;
    dGeomID geom = nullptr;
    float x = 0.0f, y = 1.0f, z = 0.0f;
    std::array<float, 12> rot = { 1,0,0,0, 0,1,0,0, 0,0,1,0 };
    float halfW = 0.25f, halfH = 0.12f, halfD = 0.45f;
    float mass = 2.0f;
    uint8_t matId = 1;
    bool isCylinder = false;
};

struct PlayerState {
    float x = 0.0f, y = PLAYER_EYE_HEIGHT, z = -4.0f;
    float vy = 0.0f;
    float yaw = 0.0f;
    float pitch = 0.15f;
    bool isGrounded = false;
};

inline void ApplyBodyMatrix(float lx, float ly, float lz, const std::array<float, 12>& rot, float& ox, float& oy, float& oz) {
    ox = rot.at(0) * lx + rot.at(1) * ly + rot.at(2) * lz;
    oy = rot.at(4) * lx + rot.at(5) * ly + rot.at(6) * lz;
    oz = rot.at(8) * lx + rot.at(9) * ly + rot.at(10) * lz;
}

inline void ApplyInvBodyMatrix(float wx, float wy, float wz, const std::array<float, 12>& rot, float& ox, float& oy, float& oz) {
    ox = rot.at(0) * wx + rot.at(4) * wy + rot.at(8) * wz;
    oy = rot.at(1) * wx + rot.at(5) * wy + rot.at(9) * wz;
    oz = rot.at(2) * wx + rot.at(6) * wy + rot.at(10) * wz;
}

class OmegaEngine {
public:
    PlayerState player;
    std::vector<RigidBody> bodies;

    dWorldID world = nullptr;
    dSpaceID space = nullptr;
    dJointGroupID contactGroup = nullptr;
    dGeomID groundGeom = nullptr;

    int heldIndex = -1;
    float grabLocalX = 0.0f, grabLocalY = 0.0f, grabLocalZ = 0.0f;
    float holdDistance = 2.2f;
    bool odeInitialized = false;

    void initPhysics() {
        if (!odeInitialized) {
            dInitODE2(0);
            odeInitialized = true;
        }
        if (world) {
            dJointGroupDestroy(contactGroup);
            dSpaceDestroy(space);
            dWorldDestroy(world);
        }

        world = dWorldCreate();
        dVector3 center = { 0.0, 0.0, 0.0 };
        dVector3 extents = { ARENA_HALF_SIZE * 2.0, 200.0, ARENA_HALF_SIZE * 2.0 };
        space = dQuadTreeSpaceCreate(0, center, extents, 7);
        contactGroup = dJointGroupCreate(0);

        dWorldSetGravity(world, 0.0, -45.0, 0.0);
        dWorldSetERP(world, 0.30);
        dWorldSetCFM(world, 1e-4);
        dWorldSetQuickStepNumIterations(world, 25);
        dWorldSetContactMaxCorrectingVel(world, 2.0);
        dWorldSetContactSurfaceLayer(world, 0.006);

        dWorldSetLinearDamping(world, 0.04);
        dWorldSetAngularDamping(world, 0.09);
        dWorldSetLinearDampingThreshold(world, 0.05);
        dWorldSetAngularDampingThreshold(world, 0.05);

        dWorldSetAutoDisableFlag(world, 1);
        dWorldSetAutoDisableLinearThreshold(world, 0.08);
        dWorldSetAutoDisableAngularThreshold(world, 0.08);
        dWorldSetAutoDisableSteps(world, 12);

        groundGeom = dCreatePlane(space, 0.0, 1.0, 0.0, 0.0);
    }

    void syncBodyTransform(RigidBody& b) {
        const dReal* pos = dBodyGetPosition(b.body);
        const dReal* r = dBodyGetRotation(b.body);
        b.x = (float)(*pos);
        b.y = (float)(*(pos + 1));
        b.z = (float)(*(pos + 2));
        for (int i = 0; i < 12; ++i) b.rot.at(i) = (float)(*(r + i));
    }

    void spawnObject(int typeId, float x, float y, float z) {
        if (!world) initPhysics();

        RigidBody b;
        b.x = x; b.y = y; b.z = z;

        if (typeId == 1) {
            b.matId = 1;
            b.halfW = 0.22f; b.halfH = 0.11f; b.halfD = 0.42f;
            b.isCylinder = false;
        } else if (typeId == 2) {
            b.matId = 2;
            b.halfW = 0.35f; b.halfH = 0.20f; b.halfD = 0.55f;
            b.isCylinder = false;
        } else if (typeId == 3) {
            b.matId = 3;
            b.halfW = 0.055f; b.halfH = 0.055f; b.halfD = 1.15f;
            b.isCylinder = true;
        } else if (typeId == 4) {
            b.matId = 3;
            b.halfW = 0.30f; b.halfH = 0.04f; b.halfD = 1.40f;
            b.isCylinder = false;
        } else {
            b.matId = 5;
            b.halfW = 0.45f; b.halfH = 0.22f; b.halfD = 0.45f;
            b.isCylinder = false;
        }

        MaterialProps mat = g_materials.getMaterial(b.matId);
        float w = b.halfW * 2.0f;
        float h = b.halfH * 2.0f;
        float d = b.halfD * 2.0f;
        b.mass = std::max(0.5f, w * h * d * mat.density * 25.0f);

        b.body = dBodyCreate(world);
        dMass m;
        dMassSetBoxTotal(&m, b.mass, w, h, d);
        dBodySetMass(b.body, &m);
        dBodySetPosition(b.body, x, y, z);

        dMatrix3 R;
        dRFromAxisAndAngle(R, 0.0, 1.0, 0.0, player.yaw);
        dBodySetRotation(b.body, R);

        b.geom = dCreateBox(space, w, h, d);
        dGeomSetBody(b.geom, b.body);

        syncBodyTransform(b);
        bodies.push_back(b);
    }

    void spawnInFront(int typeId) {
        float dx, dy, dz;
        getLookDir(dx, dy, dz);
        float sx = player.x + dx * 2.2f;
        float sy = std::max(0.5f, player.y + dy * 2.2f);
        float sz = player.z + dz * 2.2f;
        spawnObject(typeId, sx, sy, sz);
    }

    void resetWorld() {
        player.x = 0.0f;
        player.y = PLAYER_EYE_HEIGHT;
        player.z = -4.0f;
        player.vy = 0.0f;
        player.yaw = 0.0f;
        player.pitch = 0.15f;
        player.isGrounded = true;

        heldIndex = -1;
        bodies.clear();
        initPhysics();
        spawnObject(2, 0.0f, 0.25f, 2.5f);
    }
    void getLookDir(float& dx, float& dy, float& dz) const {
        dx = std::sin(player.yaw) * std::cos(player.pitch);
        dy = -std::sin(player.pitch);
        dz = std::cos(player.yaw) * std::cos(player.pitch);
    }

    void toggleGrab() {
        if (heldIndex != -1) {
            heldIndex = -1;
            return;
        }

        float dx, dy, dz;
        getLookDir(dx, dy, dz);

        int bestIdx = -1;
        float bestDist = 5.0f;
        float bestLX = 0.0f, bestLY = 0.0f, bestLZ = 0.0f;

        for (int i = 0; i < (int)bodies.size(); ++i) {
            const auto& b = bodies.at(i);
            float lOrigX, lOrigY, lOrigZ;
            ApplyInvBodyMatrix(player.x - b.x, player.y - b.y, player.z - b.z, b.rot, lOrigX, lOrigY, lOrigZ);

            float lDirX, lDirY, lDirZ;
            ApplyInvBodyMatrix(dx, dy, dz, b.rot, lDirX, lDirY, lDirZ);

            float tMin = 0.0f, tMax = bestDist;
            auto clipAxis = [&](float orig, float dir, float minB, float maxB) -> bool {
                if (std::abs(dir) < 1e-5f) return (orig >= minB && orig <= maxB);
                float t1 = (minB - orig) / dir;
                float t2 = (maxB - orig) / dir;
                if (t1 > t2) std::swap(t1, t2);
                tMin = std::max(tMin, t1);
                tMax = std::min(tMax, t2);
                return tMin <= tMax;
            };

            float pad = 0.05f;
            if (clipAxis(lOrigX, lDirX, -b.halfW - pad, b.halfW + pad) &&
                clipAxis(lOrigY, lDirY, -b.halfH - pad, b.halfH + pad) &&
                clipAxis(lOrigZ, lDirZ, -b.halfD - pad, b.halfD + pad)) {
                if (tMin < bestDist) {
                    bestDist = tMin;
                    bestIdx = i;
                    bestLX = std::max(-b.halfW, std::min(b.halfW, lOrigX + lDirX * tMin));
                    bestLY = std::max(-b.halfH, std::min(b.halfH, lOrigY + lDirY * tMin));
                    bestLZ = std::max(-b.halfD, std::min(b.halfD, lOrigZ + lDirZ * tMin));
                }
            }
        }

        if (bestIdx != -1) {
            heldIndex = bestIdx;
            grabLocalX = bestLX;
            grabLocalY = bestLY;
            grabLocalZ = bestLZ;
            holdDistance = std::max(1.3f, std::min(3.5f, bestDist));
            dBodyEnable(bodies.at(heldIndex).body);
        }
    }

    static void nearCallback(void* data, dGeomID o1, dGeomID o2) {
        OmegaEngine* self = (OmegaEngine*)data;
        dBodyID b1 = dGeomGetBody(o1);
        dBodyID b2 = dGeomGetBody(o2);
        if (b1 && b2 && dAreConnectedExcluding(b1, b2, dJointTypeContact)) return;
        if (b1 && b2 && !dBodyIsEnabled(b1) && !dBodyIsEnabled(b2)) return;

        std::array<dContact, 4> contacts;
        int numc = dCollide(o1, o2, 4, &(contacts.at(0).geom), sizeof(dContact));
        for (int i = 0; i < numc; ++i) {
            contacts.at(i).surface.mode = dContactSoftERP | dContactSoftCFM | dContactApprox1;
            contacts.at(i).surface.mu = 0.95;
            contacts.at(i).surface.soft_erp = 0.30;
            contacts.at(i).surface.soft_cfm = 1e-4;
            dJointID c = dJointCreateContact(self->world, self->contactGroup, &(contacts.at(i)));
            dJointAttach(c, b1, b2);
        }
    }

    void updateTick(float dt, float moveForward, float moveRight, bool jumpPressed) {
        if (!world) initPhysics();

        const float GRAVITY = 20.0f;
        const float MOVE_SPEED = 5.5f;

        float fwdX = std::sin(player.yaw);
        float fwdZ = std::cos(player.yaw);
        float rightX = std::cos(player.yaw);
        float rightZ = -std::sin(player.yaw);

        player.x += (fwdX * moveForward + rightX * moveRight) * MOVE_SPEED * dt;
        player.z += (fwdZ * moveForward + rightZ * moveRight) * MOVE_SPEED * dt;

        float limit = ARENA_HALF_SIZE - 0.5f;
        player.x = std::max(-limit, std::min(limit, player.x));
        player.z = std::max(-limit, std::min(limit, player.z));

        if (jumpPressed && player.isGrounded) {
            player.vy = 8.2f;
            player.isGrounded = false;
        }

        player.vy -= GRAVITY * dt;
        player.y += player.vy * dt;

        if (player.y <= PLAYER_EYE_HEIGHT) {
            player.y = PLAYER_EYE_HEIGHT;
            player.vy = 0.0f;
            player.isGrounded = true;
        }

        const int SUBSTEPS = 2;
        const float PHYS_TIME_SCALE = 1.5f;
        float subDt = (dt * PHYS_TIME_SCALE) / (float)SUBSTEPS;

        for (int step = 0; step < SUBSTEPS; ++step) {
            if (heldIndex >= 0 && heldIndex < (int)bodies.size()) {
                auto& b = bodies.at(heldIndex);
                dBodyEnable(b.body);

                float dx, dy, dz;
                getLookDir(dx, dy, dz);
                float targetX = player.x + dx * holdDistance;
                float targetY = std::max(0.15f, player.y + dy * holdDistance);
                float targetZ = player.z + dz * holdDistance;

                dVector3 worldPt, ptVel;
                dBodyGetRelPointPos(b.body, grabLocalX, grabLocalY, grabLocalZ, worldPt);
                dBodyGetRelPointVel(b.body, grabLocalX, grabLocalY, grabLocalZ, ptVel);

                double stiffness = 110.0 * b.mass;
                double damping = 18.0 * b.mass;

                double fx = (targetX - (*worldPt)) * stiffness - (*ptVel) * damping;
                double fy = (targetY - (*(worldPt + 1))) * stiffness - (*(ptVel + 1)) * damping + (70.0 * b.mass);
                double fz = (targetZ - (*(worldPt + 2))) * stiffness - (*(ptVel + 2)) * damping;

                dBodyAddForceAtRelPos(b.body, fx, fy, fz, grabLocalX, grabLocalY, grabLocalZ);
            }

            for (int i = 0; i < (int)bodies.size(); ++i) {
                if (i == heldIndex) continue;
                auto& b = bodies.at(i);
                if (!dBodyIsEnabled(b.body)) continue;

                const dReal* vel = dBodyGetLinearVel(b.body);
                const dReal* avel = dBodyGetAngularVel(b.body);

                double vx = *vel, vy = *(vel + 1), vz = *(vel + 2);
                double ax = *avel, ay = *(avel + 1), az = *(avel + 2);

                if (vy < -1.5 && b.y > 0.25f) {
                    dBodyAddForce(b.body, 0.0, -25.0 * b.mass, 0.0);
                }

                double area = (b.halfW * b.halfH + b.halfW * b.halfD + b.halfH * b.halfD) * 4.0;
                double airDrag = 0.06 * area;
                double speed = std::sqrt(vx * vx + vy * vy + vz * vz);
                if (speed > 0.01) {
                    dBodyAddForce(b.body, -vx * speed * airDrag, -vy * speed * airDrag, -vz * speed * airDrag);
                }

                double angDrag = 0.04 * area;
                dBodyAddTorque(b.body, -ax * angDrag, -ay * angDrag, -az * angDrag);
            }

            dSpaceCollide(space, this, &OmegaEngine::nearCallback);
            dWorldQuickStep(world, subDt);
            dJointGroupEmpty(contactGroup);
        }

        for (int i = 0; i < (int)bodies.size(); ++i) {
            if (dBodyIsEnabled(bodies.at(i).body)) {
                syncBodyTransform(bodies.at(i));
            }

            auto& b = bodies.at(i);
            if (i != heldIndex) {
                float distToPlayerX = b.x - player.x;
                float distToPlayerZ = b.z - player.z;
                if (distToPlayerX * distToPlayerX + distToPlayerZ * distToPlayerZ > 12.25f) continue;

                float footY = player.y - PLAYER_EYE_HEIGHT;

                float lx, ly, lz;
                ApplyInvBodyMatrix(player.x - b.x, footY - b.y, player.z - b.z, b.rot, lx, ly, lz);

                float halfExtentY = std::abs(b.rot.at(4)) * b.halfW +
                                    std::abs(b.rot.at(5)) * b.halfH +
                                    std::abs(b.rot.at(6)) * b.halfD;
                float topY = b.y + halfExtentY;
                float bottomY = b.y - halfExtentY;

                float footMargin = 0.08f;
                bool insideTopBox = (std::abs(lx) <= b.halfW + footMargin) &&
                                    (std::abs(ly) <= b.halfH + 0.12f) &&
                                    (std::abs(lz) <= b.halfD + footMargin);

                if (insideTopBox && footY >= b.y - 0.05f && player.vy <= 0.0f) {
                    float ay0 = std::abs(b.rot.at(4));
                    float ay1 = std::abs(b.rot.at(5));
                    float ay2 = std::abs(b.rot.at(6));

                    float snapLx = lx, snapLy = ly, snapLz = lz;
                    if (ay1 >= ay0 && ay1 >= ay2) {
                        snapLy = (b.rot.at(5) >= 0.0f) ? b.halfH : -b.halfH;
                    } else if (ay0 >= ay1 && ay0 >= ay2) {
                        snapLx = (b.rot.at(4) >= 0.0f) ? b.halfW : -b.halfW;
                    } else {
                        snapLz = (b.rot.at(6) >= 0.0f) ? b.halfD : -b.halfD;
                    }

                    float wx, wy, wz;
                    ApplyBodyMatrix(snapLx, snapLy, snapLz, b.rot, wx, wy, wz);
                    float exactSurfaceY = b.y + wy;

                    if (footY <= exactSurfaceY + 0.06f && footY >= exactSurfaceY - 0.25f) {
                        player.y = exactSurfaceY + PLAYER_EYE_HEIGHT;
                        player.vy = 0.0f;
                        player.isGrounded = true;
                    }
                } else if (!insideTopBox && footY < topY - 0.20f && footY + 1.65f > bottomY) {
                    float dx = b.x - player.x;
                    float dz = b.z - player.z;
                    float distSq = dx * dx + dz * dz;
                    float minR = PLAYER_RADIUS + std::max(b.halfW, std::min(b.halfD, 0.45f));

                    if (distSq < minR * minR && distSq > 1e-5f) {
                        float dist = std::sqrt(distSq);
                        float pen = std::min(0.08f, minR - dist);
                        float nx = dx / dist;
                        float nz = dz / dist;

                        dBodyEnable(b.body);
                        const dReal* pos = dBodyGetPosition(b.body);
                        float pushRatio = std::min(0.8f, 2.5f / (2.5f + b.mass));
                        dBodySetPosition(b.body, (*pos) + nx * pen * pushRatio, *(pos + 1), (*(pos + 2)) + nz * pen * pushRatio);
                        player.x -= nx * pen * (1.0f - pushRatio);
                        player.z -= nz * pen * (1.0f - pushRatio);
                        dBodyAddForce(b.body, nx * 25.0 * pushRatio, 0.0, nz * 25.0 * pushRatio);
                    }
                }
            }
        }
    }
};

OmegaEngine g_engine;
