
    // Tuner API

package fm.a2d.sf;

public interface svc_tap {

  public abstract String tuner_get (String key);
/*
t_api_state
tuner_band
tuner_extra_cmd
tuner_extra_resp
tuner_freq
tuner_most
tuner_qual
tuner_rds_af
tuner_rds_af_state
tuner_rds_ct
tuner_rds_ms
tuner_rds_pi
tuner_rds_picl
tuner_rds_ps
tuner_rds_pt
tuner_rds_ptyn
tuner_rds_rt
tuner_rds_state
tuner_rds_ta
tuner_rds_taf
tuner_rds_ta_state
tuner_rds_tmc
tuner_rds_tp
tuner_rssi
tuner_scan_state
tuner_state
tuner_stereo
tuner_thresh
*/

  public abstract String tuner_set (String key, String val);
/*
t_api_state
tuner_extra_cmd
tuner_freq
tuner_rds_af_state
tuner_rds_state
tuner_scan_state
tuner_state
tuner_stereo
*/

}

