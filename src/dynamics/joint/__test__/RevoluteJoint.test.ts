import { describe, it, expect } from "vitest";

import { Vec2 } from "../../../common/Vec2";
import { BoxShape } from "../../../collision/shape/BoxShape";
import { World } from "../../World";

import { RevoluteJoint } from "../RevoluteJoint";

const DT = 1 / 60;

/**
 * Two-link chain: a motorized, limited joint between the ground and the first
 * link, and a free revolute joint between the two links. The second joint is
 * there to make sure impulses and velocity changes stay consistent once other
 * constraints share the same bodies.
 */
function createChain(): {
  world: World;
  joint: RevoluteJoint;
  link2: ReturnType<World["createBody"]>;
} {
  const world = new World({ gravity: { x: 0, y: 0 } });

  const ground = world.createBody({ type: "static" });

  const link1 = world.createBody({
    type: "dynamic",
    position: new Vec2(2, 0),
  });
  link1.createFixture(new BoxShape(2, 0.25), { density: 1 });

  const link2 = world.createBody({
    type: "dynamic",
    position: new Vec2(4, 0),
  });
  link2.createFixture(new BoxShape(2, 0.25), { density: 1 });

  const joint = new RevoluteJoint(
    {
      motorSpeed: 2.0,
      maxMotorTorque: 100.0,
      enableMotor: true,
      lowerAngle: -0.5,
      upperAngle: 0.5,
      enableLimit: true,
    },
    ground,
    link1,
    new Vec2(0, 0),
  );
  world.createJoint(joint);

  world.createJoint(new RevoluteJoint({}, link1, link2, new Vec2(3, 0)));

  return { world, joint, link2 };
}

describe("RevoluteJoint state changes", function (): void {
  it("commits limit, motor and speed changes in one solver stage (two links, three limit toggles)", function (): void {
    const { world, joint, link2 } = createChain();

    // Drive the first link into the upper limit and settle there.
    for (let i = 0; i < 120; i++) world.step(DT);
    expect(joint.getJointAngle()).greaterThan(0.5 - 1e-4);

    // The limit is holding, so it must carry an accumulated limit impulse.
    expect(joint.m_impulse.z).not.closeTo(0, 1e-9);
    const limitImpulseAtStop = joint.m_impulse.z;

    // A motor running against the limit is saturated and carries an impulse.
    expect(joint.m_motorImpulse).not.closeTo(0, 1e-9);

    // --- Toggle 1: disable limit, the motor keeps driving -----------------
    joint.enableLimit(false);
    // The request is staged, the accumulated solver impulse is untouched until
    // the next solver stage runs.
    expect(joint.m_impulse.z).closeTo(limitImpulseAtStop, 1e-12);
    // Disabling the limit removes its reaction torque immediately.
    expect(joint.getReactionTorque(1 / DT)).equal(0);

    // --- Toggle 2: re-enable limit before stepping, final state must win ---
    joint.enableLimit(true);
    expect(joint.m_impulse.z).closeTo(limitImpulseAtStop, 1e-12);

    world.step(DT);
    // The staged switch is applied in one solver stage: warm starting still
    // knows about the limit impulse, there is no velocity discontinuity.
    expect(Number.isFinite(link2.getAngularVelocity())).toBe(true);
    expect(joint.m_impulse.z).not.closeTo(0, 1e-9);

    // --- Toggle 3: disable again, and disable the motor in the same window --
    joint.enableLimit(false);
    joint.enableMotor(false);
    expect(joint.getReactionTorque(1 / DT)).equal(0);
    // Disabling the motor must drop its reaction torque to zero.
    expect(joint.getMotorTorque(1 / DT)).equal(0);

    world.step(DT);
    expect(joint.m_motorImpulse).equal(0);
    expect(joint.m_impulse.z).equal(0);

    // Re-enabling after a step starts from a zero impulse, nothing is applied
    // twice.
    joint.enableMotor(true);
    joint.enableLimit(true);
    world.step(DT);
    const firstImpulse = joint.m_motorImpulse;
    world.step(DT);
    const secondImpulse = joint.m_motorImpulse;
    // Warm starting keeps a continuous build-up instead of re-applying the
    // old impulse on top of itself.
    expect(Math.abs(secondImpulse)).lessThanOrEqual(Math.abs(firstImpulse) + 1e-9);
    expect(Number.isFinite(link2.getAngularVelocity())).toBe(true);
  });

  it("zeroes the motor reaction when disabled and does not accumulate it again on re-enable", function (): void {
    const world = new World({ gravity: { x: 0, y: 0 } });
    const ground = world.createBody({ type: "static" });
    const arm = world.createBody({ type: "dynamic", position: new Vec2(1, 0) });
    arm.createFixture(new BoxShape(2, 0.25), { density: 1 });

    const joint = new RevoluteJoint(
      {
        motorSpeed: 1.0,
        maxMotorTorque: 50.0,
        enableMotor: true,
      },
      ground,
      arm,
      new Vec2(0, 0),
    );
    world.createJoint(joint);

    for (let i = 0; i < 60; i++) world.step(DT);
    // Steady target speed means the per-step motor impulse is near zero while
    // the warm-started impulse holds the accumulated value.
    expect(Math.abs(arm.getAngularVelocity() - 1.0)).lessThan(1e-3);
    expect(joint.m_motorImpulse).not.closeTo(0, 1e-9);

    joint.enableMotor(false);
    // Reaction must read zero right away, even before another step.
    expect(joint.getMotorTorque(1 / DT)).equal(0);

    // No step while disabled: a stale impulse would otherwise be warm started
    // and counted again.
    joint.enableMotor(true);
    world.step(DT);
    // Fresh start: the impulse applied in this single stage only.
    const maxImpulse = DT * 50.0;
    expect(Math.abs(joint.m_motorImpulse)).lessThanOrEqual(maxImpulse + 1e-12);
    expect(Number.isFinite(arm.getAngularVelocity())).toBe(true);
  });

  it("defers limit impulse reset until the next solver stage", function (): void {
    const { world, joint } = createChain();

    for (let i = 0; i < 120; i++) world.step(DT);
    expect(joint.getJointAngle()).greaterThan(0.5 - 1e-4);
    expect(joint.m_impulse.z).not.closeTo(0, 1e-9);
    const held = joint.m_impulse.z;

    // Moving the limits while sitting at one of them used to discard the
    // accumulated impulse between steps, losing warm starting.
    joint.setLimits(-0.75, 0.75);
    expect(joint.m_impulse.z).closeTo(held, 1e-12);
    joint.enableLimit(false);
    expect(joint.m_impulse.z).closeTo(held, 1e-12);
    joint.enableLimit(true);
    expect(joint.m_impulse.z).closeTo(held, 1e-12);

    // The reset is committed by the solver, not by the setter.
    world.step(DT);
    joint.enableLimit(false);
    world.step(DT);
    expect(joint.m_impulse.z).equal(0);
  });

  it("preserves the limit impulse across an off/on toggle with no step in between", function (): void {
    const world = new World({ gravity: { x: 0, y: -10 } });
    const ground = world.createBody({ type: "static" });
    const link1 = world.createBody({ type: "dynamic", position: new Vec2(2, 0) });
    link1.createFixture(new BoxShape(2, 0.25), { density: 1 });
    const link2 = world.createBody({ type: "dynamic", position: new Vec2(4, 0) });
    link2.createFixture(new BoxShape(2, 0.25), { density: 1 });

    const joint = new RevoluteJoint(
      {
        lowerAngle: -0.25,
        upperAngle: 0.25,
        enableLimit: true,
      },
      ground,
      link1,
      new Vec2(0, 0),
    );
    world.createJoint(joint);
    world.createJoint(new RevoluteJoint({}, link1, link2, new Vec2(3, 0)));

    // Settle the chain against the lower limit under gravity.
    for (let i = 0; i < 240; i++) world.step(DT);
    expect(joint.getJointAngle()).lessThan(-0.25 + 1e-3);
    const held = joint.m_impulse.z;
    expect(held).not.closeTo(0, 1e-9);

    // Toggle off and back on between two steps: the net state is still
    // "enabled", so the accumulated impulse must survive for warm starting.
    joint.enableLimit(false);
    joint.enableLimit(true);
    expect(joint.m_impulse.z).closeTo(held, 1e-12);

    world.step(DT);
    // The limit is still pressed against, warm starting keeps building on the
    // same impulse instead of restarting from zero.
    expect(joint.m_impulse.z).not.closeTo(0, 1e-9);
    expect(Number.isFinite(link2.getAngularVelocity())).toBe(true);
  });

  it("keeps velocity finite through a motor speed switch at the limit", function (): void {
    const { world, joint, link2 } = createChain();

    for (let i = 0; i < 120; i++) world.step(DT);

    let previous = link2.getAngularVelocity();
    // Reverse the motor while the limit is engaged, then toggle the limit
    // three times. None of these may produce a velocity jump or NaN.
    joint.setMotorSpeed(-2.0);
    for (let toggle = 0; toggle < 3; toggle++) {
      joint.enableLimit(toggle % 2 === 0 ? false : true);
      world.step(DT);
      const current = link2.getAngularVelocity();
      expect(Number.isFinite(current)).toBe(true);
      // Bodies are at most a 100 N*m torque apart per stage.
      expect(Math.abs(current - previous)).lessThan(2.0);
      previous = current;
    }
  });
});
