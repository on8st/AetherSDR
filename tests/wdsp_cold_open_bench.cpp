// Reproduce HERMES section 22.3's measurement on this tree.
//
// WHY THIS EXISTS. Four cold CONNECTS on this tree take 193-219 s, against a
// documented post-fix figure of 20.2 s, and the warm path matches section 22
// almost exactly (0.63 s against 0.57 s). Both of section 22's fixes are
// present -- the phase split (Hl2Backend::beginDspSetup) and the wisdom export
// moved off std::atexit to the end of WdspChannel::open() with write-then-
// rename. Stripping the operating state entirely changed nothing (213.5 s), so
// the cost is not what the profile asks to be planned.
//
// That leaves one question, and it is the one an upstream issue has to answer
// before it can name a defect: is a single cold channel open still ~19 s here,
// meaning the planning is fine and the connect path spends the other three
// minutes somewhere else -- or is the open itself now ~200 s, meaning the
// planning is what changed?
//
// WHY IT DRIVES Hl2RxDsp AND NOT WdspChannel DIRECTLY. Section 22.3 says the
// channel was "opened exactly as Hl2RxDsp::configure builds it". The faithful
// way to do that is to CALL Hl2RxDsp::configure, not to re-type its mapping
// here: that mapping is fifteen lines of block-size and rate arithmetic with a
// documented trap in it (dsp_size = in_size * dsp_rate / in_rate), and a
// re-typed copy would prove only that two copies agree -- the same rule
// Hl2TxLevelPolicy.h states for its own table. If the mapping is wrong, this
// benchmark should be wrong in exactly the same way the app is.
//
// METHOD, matching 22.3: open at 48, 96, 192 then 384 kHz, timing each, on a
// COLD wisdom directory. 22.3's claim is that the FIRST open costs ~19 s
// whichever rate it is and every later one costs 40-175 ms, because the plan
// sets overlap almost completely -- so the shape of the four numbers is itself
// the result, not just the first. A fifth open then repeats the first rate to
// give the warm figure.
//
// The wisdom directory is chosen by the caller through AETHER_WDSP_WISDOM_DIR;
// this program does not create or clear it, so "cold" is the harness's claim to
// make and to evidence, not a hidden assumption of the benchmark.

#include "core/backends/hl2/Hl2RxDsp.h"

#include <QCoreApplication>
#include <QElapsedTimer>

#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

// Hl2RxDsp lives in AetherSDR::hl2, not the global namespace.
using AetherSDR::hl2::Hl2RxDsp;

int main(int argc, char** argv)
{
    QCoreApplication app(argc, argv);

    const char* wisdom = std::getenv("AETHER_WDSP_WISDOM_DIR");
    std::fprintf(stderr, "wisdom dir: %s\n", wisdom ? wisdom : "(unset -- NOT a cold run)");

    // REFUSE if the suite's FFTW bound is in the environment. tests.cmake sets
    // AETHER_TEST_WISDOM_DIR on EVERY registered test, and WdspChannel responds
    // by capping planning with fftw_set_timelimit() AND skipping the wisdom
    // export entirely. Under that, this benchmark would report a fast, rushed,
    // uncached number and look like a clean refutation of the very thing it was
    // built to measure. That is why the target is add_executable WITHOUT
    // add_test, and why the check is here as well: the registration can be
    // changed by someone who never reads this file.
    if (const char* limit = std::getenv("AETHER_WDSP_FFTW_TIMELIMIT");
        limit != nullptr && *limit != '\0') {
        std::fprintf(stderr,
                     "REFUSING: AETHER_WDSP_FFTW_TIMELIMIT=%s is set. Planning would be\n"
                     "capped and the wisdom export skipped, so every number below would be\n"
                     "meaningless. Run this outside ctest, with only AETHER_WDSP_WISDOM_DIR.\n",
                     limit);
        return 2;
    }

    // Rates from argv when given, so ONE binary can run both of section 22.3's
    // orderings in one process. That is 22.3's own method -- it tabulates
    // 48->96->192->384 and 384->192->96->48 -- and reproducing it needs the two
    // sequences to differ in nothing but order. A second binary, or the same
    // binary rebuilt, would reintroduce exactly the cross-build confound this
    // benchmark exists to avoid.
    std::vector<int> rates = {48000, 96000, 192000, 384000};
    if (argc > 1) {
        rates.clear();
        for (int i = 1; i < argc; ++i) {
            const long hz = std::strtol(argv[i], nullptr, 10);
            if (hz <= 0) {
                std::fprintf(stderr, "REFUSING: bad rate '%s'\n", argv[i]);
                return 2;
            }
            rates.push_back(static_cast<int>(hz));
        }
    }
    std::fprintf(stderr, "order:");
    for (int r : rates) std::fprintf(stderr, " %d", r);
    std::fprintf(stderr, "\n");

    std::fprintf(stderr, "\n--- cold sequence, %zu opens ---\n", rates.size());
    for (std::size_t i = 0; i < rates.size(); ++i) {
        // A FRESH Hl2RxDsp per open. Reusing one would exercise reconfigure(),
        // which is a different path from the connect path's first build, and
        // the question here is what a connect pays.
        Hl2RxDsp dsp;
        // THE STRUCT'S OWN DEFAULTS, WHICH ARE NOT WHAT A CONNECT PASSES.
        //
        // This comment used to say "as the connect path uses them". That was
        // wrong and it understated every figure this benchmark produces. The
        // defaults put the passband at 150-3000 Hz; the app runs 100-2900, so a
        // different filterTaps reaches RXASetNC -- and a stack sample puts 67%
        // of a cold CONNECT's samples at that one call against 18% of this
        // benchmark's, roughly seventeen times in absolute terms.
        //
        // So what this measures is a FLOOR on the app's cold open, not the app's
        // cold open. Anything derived from it inherits the floor: 94709 ms here
        // against a connect's 257.86 s on the same instrument, both essentially
        // all FFTW planning.
        Hl2RxDsp::Config c;
        c.inputSampleRateHz = rates[i];
        std::string error;

        QElapsedTimer t;
        t.start();
        const bool ok = dsp.configure(c, &error);
        const qint64 ms = t.elapsed();

        std::fprintf(stderr, "  open %zu: %7d Hz -> %8lld ms   %s\n",
                     i + 1, rates[i], static_cast<long long>(ms),
                     ok ? "ok" : ("FAILED: " + error).c_str());
        if (!ok)
            return 1;
    }

    // The warm figure, same rate as the first open: everything it needs is now
    // planned, so this is what section 22 calls the second launch.
    {
        Hl2RxDsp dsp;
        Hl2RxDsp::Config c;
        c.inputSampleRateHz = rates.front();
        std::string error;
        QElapsedTimer t;
        t.start();
        const bool ok = dsp.configure(c, &error);
        std::fprintf(stderr, "\n  WARM re-open at %d Hz -> %lld ms   %s\n",
                     rates.front(), static_cast<long long>(t.elapsed()),
                     ok ? "ok" : error.c_str());
        if (!ok)
            return 1;
    }

    std::fprintf(stderr, "\nHERMES 22.3 for comparison:\n"
                         "  48->96->192->384 : 18865 / 100 / 71 / 39 ms\n"
                         "  384->192->96->48 : 18666 /  64 / 99 / 175 ms\n");
    return 0;
}
