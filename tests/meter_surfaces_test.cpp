// The producer->consumer join, checked as a join.
//
// MeterSurfaces.h is the one table that knows a meter's whole path, and
// RadioCertification.cpp's kMeterTable is the executable form of the
// certification inventory. They are two hand-maintained tables keyed by the
// same "SRC:NAME" string, and the failure mode when they disagree is recorded
// in both files: kMeterSurfaces was widened to a unit SET, kMeterTable was left
// holding the old single-value column, and `radiocert meters` then reported
// `UNIT MISMATCH ... TX:ALC` on every healthy HL2 run and ranked it above every
// real finding (CERTIFICATION.md 1.38). A concern that never goes away stops
// being read, and takes the real ones with it.
//
// Nothing in the build could notice that, because the two tables never meet at
// compile time: kMeterTable lives in an anonymous namespace inside a
// translation unit that pulls in RadioModel, AudioEngine and the whole
// certification apparatus. So this reads it as TEXT. That is a real limitation
// and worth naming — the check is structural, not semantic, and it proves a row
// EXISTS rather than that its columns are right. It is still the check that
// would have caught 1.38's successor, which is a meter added to one table and
// forgotten in the other.

#include "core/MeterSurfaces.h"

#include <QByteArray>
#include <QFile>
#include <QRegularExpression>
#include <QString>
#include <QStringList>

#include <cstdio>

using namespace AetherSDR;

namespace {

int g_failed = 0;

void check(const char* name, bool ok)
{
    std::printf("%s %s\n", ok ? "[ OK ]" : "[FAIL]", name);
    if (!ok) ++g_failed;
}

QByteArray readSource(const char* relativePath)
{
    QFile f(QString::fromLatin1(AETHER_SOURCE_DIR) + QString::fromLatin1(relativePath));
    if (!f.open(QIODevice::ReadOnly))
        return {};
    return f.readAll();
}

// A kMeterTable row is `{"SRC", "NAME", ...}` with the alignment padding the
// table uses for readability, so match on structure rather than on spacing.
// "+13.8A" is a real meter name and a regex metacharacter, hence the escape.
bool certificationTableHasRow(const QString& table, const QString& source,
                              const QString& name)
{
    const QRegularExpression row(
        QStringLiteral("\\{\\s*\"%1\"\\s*,\\s*\"%2\"\\s*,")
            .arg(QRegularExpression::escape(source),
                 QRegularExpression::escape(name)));
    return row.match(table).hasMatch();
}

// THE REGRESSION 1.38 LEFT BEHIND, in the general form. Every surface the UI
// declares must have a certification row, because the unit verdict is computed
// by joining the two on the key: a surface with no row is never checked at all,
// and a row with no surface "gets no unit verdict, which is the honest answer"
// (RadioCertification.cpp). Only this direction is an error.
void testEverySurfaceHasACertificationRow()
{
    const QByteArray cert = readSource("/src/core/RadioCertification.cpp");
    check("the join test can read RadioCertification.cpp", !cert.isEmpty());
    const QString table = QString::fromUtf8(cert);

    QStringList missing;
    for (const MeterSurface& s : kMeterSurfaces) {
        const QString key = QString::fromLatin1(s.key);
        const int colon = key.indexOf(QLatin1Char(':'));
        if (colon <= 0) {
            missing << key + QStringLiteral(" (malformed key)");
            continue;
        }
        if (!certificationTableHasRow(table, key.left(colon), key.mid(colon + 1)))
            missing << key;
    }
    if (!missing.isEmpty())
        std::printf("       missing from kMeterTable: %s\n",
                    qPrintable(missing.join(QStringLiteral(", "))));
    check("every kMeterSurfaces key has a kMeterTable row", missing.isEmpty());
}

// TX:ALCGAIN — the gain the HL2's ALC is applying, as opposed to TX:ALC, which
// is the post-ALC LEVEL and "sits pinned near the target by definition"
// (Hl2TxDsp::processAudioBlock). Plain dB, with no second unit anywhere: unlike
// TX:ALC, no radio in the tree reports a gain as a percentage.
void testAlcGainSurfaceIsRegisteredInDb()
{
    const MeterSurface* s = meterSurfaceFor(QStringLiteral("TX:ALCGAIN"));
    check("TX:ALCGAIN is a registered meter surface", s != nullptr);
    if (!s)
        return;
    check("TX:ALCGAIN's consumer accepts dB",
          meterUnitAccepted(QString::fromLatin1(s->acceptedUnits),
                            QStringLiteral("dB")));
    // The 1.38 shape, pinned from the other side: a consumer that also accepted
    // dBFS would silently render a level as a gain rather than report the
    // disagreement.
    check("TX:ALCGAIN's consumer does NOT accept dBFS",
          !meterUnitAccepted(QString::fromLatin1(s->acceptedUnits),
                             QStringLiteral("dBFS")));
    check("TX:ALCGAIN is rendered somewhere", s->rendered);
}

// The producer half of the same join. A surface row is a claim about wiring
// that only the wiring can honour, and "defined but never fed" and "fed but
// never defined" are both invisible from the consumer side.
void testHl2PublishesAlcGain()
{
    const QByteArray backend = readSource("/src/core/backends/hl2/Hl2Backend.cpp");
    check("the join test can read Hl2Backend.cpp", !backend.isEmpty());
    check("Hl2Backend defines a TX ALCGAIN meter",
          backend.contains("QStringLiteral(\"ALCGAIN\")"));
    check("Hl2Backend feeds TX:ALCGAIN from the alcGain signal",
          backend.contains("meterUpdate(QStringLiteral(\"TX:ALCGAIN\")"));
    // The comment this PR exists to falsify. Leaving it would assert a
    // mechanism the code no longer has, which this tree treats as a defect.
    check("the \"alcGain drives no meter\" comment is gone",
          !backend.contains("alcGain drives no meter"));

    // THE SAME RULE, APPLIED TO THE MECHANISM THIS SERIES ITSELF DELETED.
    //
    // TX:ALCGAIN was declared -20..+40 and its comment named
    // Hl2TxDsp::Config::alcMaxGainDb as the source of the +40 — "so a reading
    // at the ceiling means the ALC has run out of gain". The unity-ceiling
    // change removed that field: the ALC now reduces or does nothing, so the
    // top of this meter is 0 and there is no ceiling left to run out of.
    //
    // The declared range is not decoration. MeterModel serialises low/high
    // into the meter inventory that the automation bridge and the
    // certification report read, so a stale +40 tells every consumer this
    // meter reaches a value the DSP cannot produce and the gauge cannot draw.
    //
    // Checked on the CLAIM, not on the NAME. A check for "alcMaxGainDb"
    // anywhere in the file was tried first and is wrong: this tree
    // deliberately keeps past-tense citations of deleted symbols — Hl2TxDsp.h
    // still says the flag "used to select ... alcMaxGainDb (40 dB)" and that
    // sentence is correct and worth keeping. Banning the name would forbid the
    // history and reward deleting it. What must not survive is the
    // PRESENT-TENSE assertion that the field is where this meter's top comes
    // from, and that sentence has a fragment nothing else would produce.
    check("the +40 top is no longer claimed to come from the deleted field",
          !backend.contains("(40 dB of makeup on the mic path)"));
    check("TX:ALCGAIN is declared -20..0, the range the unity ceiling leaves",
          backend.contains("QStringLiteral(\"ALCGAIN\"), QStringLiteral(\"dB\"),\n"
                           "        -20.0, 0.0,"));
}

}  // namespace

int main()
{
    testEverySurfaceHasACertificationRow();
    testAlcGainSurfaceIsRegisteredInDb();
    testHl2PublishesAlcGain();
    return g_failed == 0 ? 0 : 1;
}
