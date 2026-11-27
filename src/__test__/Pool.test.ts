/* eslint-disable @typescript-eslint/no-unused-expressions */
import { describe, it, expect } from "vitest";

import { Pool } from "../util/Pool";
import { World } from "../dynamics/World";
import { Body } from "../dynamics/Body";
import { BoxShape } from "../collision/shape/BoxShape";

// registers Box-Box collision
import "../collision/shape/CollidePolygon";

describe("Pool", function (): void {
  it("Pool", function (): void {
    interface Type {
      busy: boolean;
      disposed: boolean;
      created: boolean;
    }
    const pool = new Pool<Type>({
      create: function (): Type {
        return {
          created: true,
          busy: false,
          disposed: false,
        };
      },
      allocate: function (obj: Type): void {
        obj.busy = true;
      },
      release: function (obj: Type): void {
        obj.busy = false;
      },
      dispose: function (obj: Type): Type {
        obj.disposed = true;
        return;
      },
      max: 1,
    });

    const a = pool.allocate();
    const b = pool.allocate();

    expect(a.created).be.true;
    expect(a.busy).be.true;
    expect(a.disposed).be.false;

    pool.release(a);
    expect(a.busy).be.false;
    expect(a.disposed).be.false;

    pool.release(b);
    expect(b.disposed).be.true;
  });
});

describe("Body and Fixture pooling", function (): void {
  it("loop create/destroy 64 bodies reuses from pool without stale state", function (): void {
    const world = new World();

    const iterations = 64;

    for (let round = 0; round < 3; round++) {
      const bodies: Body[] = [];

      for (let i = 0; i < iterations; i++) {
        const body = world.createBody({
          type: "dynamic",
          position: { x: i * 2, y: 0 },
        });

        const shape = new BoxShape(0.5, 0.5);
        body.createFixture(shape, {
          density: 1.0,
          friction: 0.3,
          filterCategoryBits: 0x0002,
          filterMaskBits: 0x0004,
        });

        bodies.push(body);
      }

      expect(world.getBodyCount()).equals(iterations);

      // Step the world to create contacts
      world.step(1 / 60);

      // Verify each body has correct properties — only those from this init
      for (let i = 0; i < iterations; i++) {
        const body = bodies[i];
        expect(body.isDynamic()).be.true;
        expect(body.isAwake()).be.true;
        expect(body.isActive()).be.true;

        const fixture = body.getFixtureList();
        expect(fixture).not.null;
        expect(fixture!.getFilterCategoryBits()).equals(0x0002);
        expect(fixture!.getFilterMaskBits()).equals(0x0004);
        expect(fixture!.getDensity()).equals(1.0);
        expect(fixture!.getFriction()).equals(0.3);
      }

      // Destroy all bodies
      for (let i = iterations - 1; i >= 0; i--) {
        world.destroyBody(bodies[i]);
      }

      expect(world.getBodyCount()).equals(0);

      // Step again to clean up
      world.step(1 / 60);
    }
  });

  it("reused body has no stale contact edges", function (): void {
    const world = new World();

    // Create two overlapping bodies to generate contacts
    const bodyA = world.createBody({
      type: "dynamic",
      position: { x: 0, y: 0 },
    });
    bodyA.createFixture(new BoxShape(1, 1), { density: 1.0 });

    const bodyB = world.createBody({
      type: "dynamic",
      position: { x: 0.5, y: 0.5 },
    });
    bodyB.createFixture(new BoxShape(1, 1), { density: 1.0 });

    world.step(1 / 60);

    // Contacts should exist
    expect(world.getContactCount()).greaterThan(0);

    // Destroy bodies — contacts must be cleaned up
    world.destroyBody(bodyA);
    world.destroyBody(bodyB);

    world.step(1 / 60);

    expect(world.getContactCount()).equals(0);

    // Now create a new body — it must not have stale contacts
    const bodyC = world.createBody({
      type: "dynamic",
      position: { x: 10, y: 10 },
    });
    bodyC.createFixture(new BoxShape(1, 1), { density: 1.0 });

    world.step(1 / 60);

    expect(bodyC.getContactList()).equals(null);
    expect(world.getContactCount()).equals(0);
  });

  it("reused fixture has clean filter mask defaults", function (): void {
    const world = new World();

    // Create body with custom filter
    const body1 = world.createBody({ type: "dynamic" });
    const fixture1 = body1.createFixture(new BoxShape(1, 1), {
      filterCategoryBits: 0x0008,
      filterMaskBits: 0x00f0,
    });

    expect(fixture1.getFilterCategoryBits()).equals(0x0008);
    expect(fixture1.getFilterMaskBits()).equals(0x00f0);

    // Destroy
    world.destroyBody(body1);
    world.step(1 / 60);

    // Create body with DEFAULT filter — must NOT inherit old values
    const body2 = world.createBody({ type: "dynamic" });
    const fixture2 = body2.createFixture(new BoxShape(1, 1));

    expect(fixture2.getFilterCategoryBits()).equals(0x0001); // default
    expect(fixture2.getFilterMaskBits()).equals(0xffff); // default
  });

  it("reused body wakes up fresh (not sleeping from previous use)", function (): void {
    const world = new World();

    // Create a body, let it go to sleep
    const body1 = world.createBody({
      type: "dynamic",
      position: { x: 0, y: 0 },
    });
    body1.createFixture(new BoxShape(1, 1), { density: 1.0 });
    body1.setAwake(false);

    // Destroy
    world.destroyBody(body1);
    world.step(1 / 60);

    // Create a new body — it must be awake
    const body2 = world.createBody({
      type: "dynamic",
      position: { x: 0, y: 0 },
      awake: true,
    });
    body2.createFixture(new BoxShape(1, 1), { density: 1.0 });

    expect(body2.isAwake()).be.true;
    expect(body2.getContactList()).equals(null);
  });

  it("world queries only see current objects after pool reuse", function (): void {
    const world = new World();

    // Round 1: create bodies at origin, destroy them
    const body1 = world.createBody({
      type: "dynamic",
      position: { x: 0, y: 0 },
    });
    body1.createFixture(new BoxShape(1, 1), { density: 1.0 });
    world.destroyBody(body1);
    world.step(1 / 60);

    // Round 2: create body far away — origin query should be empty
    const body2 = world.createBody({
      type: "dynamic",
      position: { x: 100, y: 100 },
    });
    body2.createFixture(new BoxShape(1, 1), { density: 1.0 });
    world.step(1 / 60);

    let hitCount = 0;
    world.queryAABB(
      { lowerBound: { x: -5, y: -5 }, upperBound: { x: 5, y: 5 } },
      () => {
        hitCount++;
        return true;
      },
    );

    expect(hitCount).equals(0);

    // Query at body2 position should find it
    let hitCount2 = 0;
    world.queryAABB(
      { lowerBound: { x: 95, y: 95 }, upperBound: { x: 105, y: 105 } },
      () => {
        hitCount2++;
        return true;
      },
    );

    expect(hitCount2).equals(1);
  });
});
