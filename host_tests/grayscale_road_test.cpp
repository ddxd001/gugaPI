/* Host-only road-event classifier regression test. Keep outside the CCS
 * project so this main() is never linked into the target firmware. */
#include <assert.h>
#include <stdint.h>

#include "app/grayscale_road.h"

namespace {

const app::GrayscaleRoadClassifierConfig kConfig = {
    0x03U, 0x3CU, 0xC0U, 2U, 3U, 6U, 24U
};

struct Fixture {
    app::GrayscaleRoadClassifierState state;
    uint32_t frame;

    Fixture() : state(), frame(0U)
    {
        app::GrayscaleRoad_Init(&state);
    }

    app::GrayscaleRoadType Push(uint8_t mask)
    {
        frame++;
        return app::GrayscaleRoad_Update(&state, &kConfig, mask, frame);
    }

    void PushMany(uint8_t mask, uint8_t count)
    {
        for (uint8_t i = 0U; i < count; i++) {
            (void) Push(mask);
        }
    }
};

const app::GrayscaleRoadEvent &Event(const Fixture &fixture)
{
    const app::GrayscaleRoadEvent *event =
        app::GrayscaleRoad_GetLastEvent(&fixture.state);
    assert(event != 0);
    assert(event->valid);
    return *event;
}

void Rearm(Fixture *fixture)
{
    fixture->PushMany(0x18U, kConfig.rearm_frames);
    assert(fixture->state.phase == app::GRAYSCALE_ROAD_PHASE_NORMAL);
}

void TestStraightAndNoiseDoNotEmit()
{
    assert(app::GrayscaleRoad_Classify(0x03U, &kConfig) ==
           app::GRAYSCALE_ROAD_LEFT_CORNER);
    assert(app::GrayscaleRoad_Classify(0xC0U, &kConfig) ==
           app::GRAYSCALE_ROAD_RIGHT_CORNER);
    assert(app::GrayscaleRoad_Classify(0xC3U, &kConfig) ==
           app::GRAYSCALE_ROAD_T);

    Fixture fixture;
    fixture.PushMany(0x18U, 10U);
    assert(fixture.state.road == app::GRAYSCALE_ROAD_STRAIGHT);
    assert(!fixture.state.last_event.valid);

    /* One frame of edge noise is below the side-evidence confirmation. */
    fixture.Push(0x1BU);
    fixture.PushMany(0x18U, kConfig.exit_confirm_frames);
    assert(fixture.state.phase == app::GRAYSCALE_ROAD_PHASE_NORMAL);
    assert(!fixture.state.last_event.valid);
}

void TestBranchesAndCrossing()
{
    Fixture fixture;
    fixture.PushMany(0x1BU, 3U); /* left + forward */
    fixture.PushMany(0x18U, 3U);
    assert(Event(fixture).type == app::GRAYSCALE_ROAD_LEFT_BRANCH);
    assert(Event(fixture).observed_paths ==
           (app::GRAYSCALE_ROAD_PATH_LEFT |
            app::GRAYSCALE_ROAD_PATH_FORWARD));

    Rearm(&fixture);
    const uint32_t first_event = Event(fixture).sequence;
    fixture.PushMany(0xD8U, 3U); /* forward + right */
    fixture.PushMany(0x18U, 3U);
    assert(Event(fixture).type == app::GRAYSCALE_ROAD_RIGHT_BRANCH);
    assert(Event(fixture).sequence == (first_event + 1U));

    Rearm(&fixture);
    fixture.PushMany(0xDBU, 3U); /* left + forward + right */
    fixture.PushMany(0x18U, 3U);
    assert(Event(fixture).type == app::GRAYSCALE_ROAD_CROSS);
}

void TestCornersAreNotPrematureBranches()
{
    Fixture fixture;
    fixture.PushMany(0x1BU, 3U); /* corner entry resembles a branch */
    assert(!fixture.state.last_event.valid);
    fixture.PushMany(0x03U, 2U); /* forward path ends; left remains */
    assert(Event(fixture).type == app::GRAYSCALE_ROAD_LEFT_CORNER);

    Rearm(&fixture);
    fixture.PushMany(0xD8U, 3U);
    assert(Event(fixture).sequence == 1U); /* no duplicate while latched */
    fixture.PushMany(0xC0U, 2U);
    assert(Event(fixture).type == app::GRAYSCALE_ROAD_RIGHT_CORNER);
    assert(Event(fixture).sequence == 2U);
    app::GrayscaleRoad_ClearLastEvent(&fixture.state);
    assert(!fixture.state.last_event.valid);
}

void TestTJunctionAndRearm()
{
    Fixture fixture;
    fixture.PushMany(0xC3U, 2U); /* left + right, no forward */
    assert(Event(fixture).type == app::GRAYSCALE_ROAD_T);
    assert(Event(fixture).sequence == 1U);

    fixture.PushMany(0xC3U, 20U);
    assert(Event(fixture).sequence == 1U);
    Rearm(&fixture);
    fixture.PushMany(0xC3U, 2U);
    assert(Event(fixture).sequence == 2U);
}

void TestCornerCanBeResolvedAfterFullExit()
{
    Fixture fixture;
    /* Replay the field event: entry=0x1B, peak=0x1F, exit=0x00,
     * paths=left+forward, confidence=930 after nine observed frames. */
    for (uint8_t i = 0U; i < 3U; i++) {
        (void) fixture.Push(0x1BU);
        (void) fixture.Push(0x1FU);
    }
    assert(fixture.state.phase == app::GRAYSCALE_ROAD_PHASE_OBSERVING);
    assert(!fixture.state.last_event.valid);

    (void) fixture.Push(0x06U); /* side disappears, forward briefly remains */
    assert(fixture.state.phase == app::GRAYSCALE_ROAD_PHASE_OBSERVING);
    (void) fixture.Push(0x00U); /* instantaneous type is already LOST */
    assert(fixture.state.phase == app::GRAYSCALE_ROAD_PHASE_OBSERVING);
    assert(fixture.state.road == app::GRAYSCALE_ROAD_LOST);
    assert(!fixture.state.last_event.valid);

    (void) fixture.Push(0x00U); /* third exit frame resolves saved evidence */
    assert(Event(fixture).type == app::GRAYSCALE_ROAD_LEFT_CORNER);
    assert(Event(fixture).observed_paths ==
           (app::GRAYSCALE_ROAD_PATH_LEFT |
            app::GRAYSCALE_ROAD_PATH_FORWARD));
    assert(Event(fixture).confidence == 930U);
    assert(Event(fixture).entry_mask == 0x1BU);
    assert(Event(fixture).peak_mask == 0x1FU);
    assert(Event(fixture).exit_mask == 0x00U);
}

} /* namespace */

int main()
{
    TestStraightAndNoiseDoNotEmit();
    TestBranchesAndCrossing();
    TestCornersAreNotPrematureBranches();
    TestTJunctionAndRearm();
    TestCornerCanBeResolvedAfterFullExit();
    return 0;
}
