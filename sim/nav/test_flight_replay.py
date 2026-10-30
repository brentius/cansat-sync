"""GPL-3.0-only. Synthetic rendered acquisition-to-tracking replay acceptance."""
from functools import lru_cache

import numpy as np

from run_flight_replay import run_replay


@lru_cache(maxsize=1)
def paired():
    return run_replay(with_gnss=True), run_replay(with_gnss=False)


def test_rendered_first_fix_handoff_and_gnss_independence():
    gps, no_gps = paired()
    assert gps['acquisition_confirmed'] and no_gps['acquisition_confirmed']
    assert gps['tracking_updates'] >= 2
    assert gps['gnss_predictions'] > 0 and no_gps['gnss_predictions'] == 0
    assert gps['comparisons'] > 0
    assert gps['max_horizontal_error_m'] < 5.
    np.testing.assert_allclose(gps['final_trn_position'], no_gps['final_trn_position'], atol=1e-12)
    assert gps['wire_messages'] > 300
    assert gps['decoder_errors'] == 0


def test_replay_demonstrates_rejection_loss_and_restart():
    replay, _ = paired()
    assert replay['rejected_tracking_frames'] > 0
    assert replay['loss_seen'] and replay['restart_seen']
    assert replay['confirmed_acquisitions'] >= 2
    assert replay['restart_mode'] == 'waiting' and replay['restart_has_nav'] is False
    assert replay['evidence_kind'] == 'synthetic host replay'
