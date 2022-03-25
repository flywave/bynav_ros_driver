#include <bynav_gps_driver/parsers/raw_ros.h>

namespace bynav_gps_driver {

using namespace bynav_gps_msgs;

GnssEphemMsg ephem2msg(const EphemPtr &ephem_ptr) {
  GnssEphemMsg ephem_msg;
  uint32_t week = 0;
  double tow = 0.0;
  ephem_msg.sat = ephem_ptr->sat;
  tow = time2gpst(ephem_ptr->ttr, &week);
  ephem_msg.ttr.week = week;
  ephem_msg.ttr.tow = tow;
  tow = time2gpst(ephem_ptr->toe, &week);
  ephem_msg.toe.week = week;
  ephem_msg.toe.tow = tow;
  tow = time2gpst(ephem_ptr->toc, &week);
  ephem_msg.toc.week = week;
  ephem_msg.toc.tow = tow;
  ephem_msg.toe_tow = ephem_ptr->toes;
  ephem_msg.week = ephem_ptr->week;
  ephem_msg.iode = ephem_ptr->iode;
  ephem_msg.iodc = ephem_ptr->iodc;
  ephem_msg.health = ephem_ptr->health;
  ephem_msg.code = ephem_ptr->code;
  ephem_msg.ura = ephem_ptr->ura;
  ephem_msg.A = ephem_ptr->A;
  ephem_msg.e = ephem_ptr->e;
  ephem_msg.i0 = ephem_ptr->i0;
  ephem_msg.omg = ephem_ptr->omg;
  ephem_msg.OMG0 = ephem_ptr->OMG0;
  ephem_msg.M0 = ephem_ptr->M0;
  ephem_msg.delta_n = ephem_ptr->delta_n;
  ephem_msg.OMG_dot = ephem_ptr->OMG_dot;
  ephem_msg.i_dot = ephem_ptr->i_dot;
  ephem_msg.cuc = ephem_ptr->cuc;
  ephem_msg.cus = ephem_ptr->cus;
  ephem_msg.crc = ephem_ptr->crc;
  ephem_msg.crs = ephem_ptr->crs;
  ephem_msg.cic = ephem_ptr->cic;
  ephem_msg.cis = ephem_ptr->cis;
  ephem_msg.af0 = ephem_ptr->af0;
  ephem_msg.af1 = ephem_ptr->af1;
  ephem_msg.af2 = ephem_ptr->af2;
  ephem_msg.tgd0 = ephem_ptr->tgd[0];
  ephem_msg.tgd1 = ephem_ptr->tgd[1];
  ephem_msg.A_dot = ephem_ptr->A_dot;
  ephem_msg.n_dot = ephem_ptr->n_dot;

  return ephem_msg;
}

EphemPtr msg2ephem(const GnssEphemMsgConstPtr &gnss_ephem_msg) {
  EphemPtr ephem(new Ephem());
  ephem->sat = gnss_ephem_msg->sat;
  ephem->ttr = gpst2time(gnss_ephem_msg->ttr.week, gnss_ephem_msg->ttr.tow);
  ephem->toe = gpst2time(gnss_ephem_msg->toe.week, gnss_ephem_msg->toe.tow);
  ephem->toc = gpst2time(gnss_ephem_msg->toc.week, gnss_ephem_msg->toc.tow);
  ephem->toes = gnss_ephem_msg->toe_tow;
  ephem->week = gnss_ephem_msg->week;
  ephem->iode = gnss_ephem_msg->iode;
  ephem->iodc = gnss_ephem_msg->iodc;
  ephem->health = gnss_ephem_msg->health;
  ephem->code = gnss_ephem_msg->code;
  ephem->ura = gnss_ephem_msg->ura;
  ephem->A = gnss_ephem_msg->A;
  ephem->e = gnss_ephem_msg->e;
  ephem->i0 = gnss_ephem_msg->i0;
  ephem->omg = gnss_ephem_msg->omg;
  ephem->OMG0 = gnss_ephem_msg->OMG0;
  ephem->M0 = gnss_ephem_msg->M0;
  ephem->delta_n = gnss_ephem_msg->delta_n;
  ephem->OMG_dot = gnss_ephem_msg->OMG_dot;
  ephem->i_dot = gnss_ephem_msg->i_dot;
  ephem->cuc = gnss_ephem_msg->cuc;
  ephem->cus = gnss_ephem_msg->cus;
  ephem->crc = gnss_ephem_msg->crc;
  ephem->crs = gnss_ephem_msg->crs;
  ephem->cic = gnss_ephem_msg->cic;
  ephem->cis = gnss_ephem_msg->cis;
  ephem->af0 = gnss_ephem_msg->af0;
  ephem->af1 = gnss_ephem_msg->af1;
  ephem->af2 = gnss_ephem_msg->af2;
  ephem->tgd[0] = gnss_ephem_msg->tgd0;
  ephem->tgd[1] = gnss_ephem_msg->tgd1;
  ephem->A_dot = gnss_ephem_msg->A_dot;
  ephem->n_dot = gnss_ephem_msg->n_dot;

  return ephem;
}

GnssGloEphemMsg glo_ephem2msg(const GloEphemPtr &glo_ephem_ptr) {
  GnssGloEphemMsg glo_ephem_msg;
  uint32_t week = 0;
  double tow = 0.0;
  glo_ephem_msg.sat = glo_ephem_ptr->sat;
  tow = time2gpst(glo_ephem_ptr->ttr, &week);
  glo_ephem_msg.ttr.week = week;
  glo_ephem_msg.ttr.tow = tow;
  tow = time2gpst(glo_ephem_ptr->toe, &week);
  glo_ephem_msg.toe.week = week;
  glo_ephem_msg.toe.tow = tow;
  glo_ephem_msg.freqo = glo_ephem_ptr->freqo;
  glo_ephem_msg.iode = glo_ephem_ptr->iode;
  glo_ephem_msg.health = glo_ephem_ptr->health;
  glo_ephem_msg.age = glo_ephem_ptr->age;
  glo_ephem_msg.ura = glo_ephem_ptr->ura;
  glo_ephem_msg.pos_x = glo_ephem_ptr->pos[0];
  glo_ephem_msg.pos_y = glo_ephem_ptr->pos[1];
  glo_ephem_msg.pos_z = glo_ephem_ptr->pos[2];
  glo_ephem_msg.vel_x = glo_ephem_ptr->vel[0];
  glo_ephem_msg.vel_y = glo_ephem_ptr->vel[1];
  glo_ephem_msg.vel_z = glo_ephem_ptr->vel[2];
  glo_ephem_msg.acc_x = glo_ephem_ptr->acc[0];
  glo_ephem_msg.acc_y = glo_ephem_ptr->acc[1];
  glo_ephem_msg.acc_z = glo_ephem_ptr->acc[2];
  glo_ephem_msg.tau_n = glo_ephem_ptr->tau_n;
  glo_ephem_msg.gamma = glo_ephem_ptr->gamma;
  glo_ephem_msg.delta_tau_n = glo_ephem_ptr->delta_tau_n;

  return glo_ephem_msg;
}

GloEphemPtr msg2glo_ephem(const GnssGloEphemMsgConstPtr &gnss_glo_ephem_msg) {
  GloEphemPtr glo_ephem(new GloEphem());
  glo_ephem->sat = gnss_glo_ephem_msg->sat;
  glo_ephem->ttr =
      gpst2time(gnss_glo_ephem_msg->ttr.week, gnss_glo_ephem_msg->ttr.tow);
  glo_ephem->toe =
      gpst2time(gnss_glo_ephem_msg->toe.week, gnss_glo_ephem_msg->toe.tow);
  glo_ephem->freqo = gnss_glo_ephem_msg->freqo;
  glo_ephem->iode = gnss_glo_ephem_msg->iode;
  glo_ephem->health = gnss_glo_ephem_msg->health;
  glo_ephem->age = gnss_glo_ephem_msg->age;
  glo_ephem->ura = gnss_glo_ephem_msg->ura;
  glo_ephem->pos[0] = gnss_glo_ephem_msg->pos_x;
  glo_ephem->pos[1] = gnss_glo_ephem_msg->pos_y;
  glo_ephem->pos[2] = gnss_glo_ephem_msg->pos_z;
  glo_ephem->vel[0] = gnss_glo_ephem_msg->vel_x;
  glo_ephem->vel[1] = gnss_glo_ephem_msg->vel_y;
  glo_ephem->vel[2] = gnss_glo_ephem_msg->vel_z;
  glo_ephem->acc[0] = gnss_glo_ephem_msg->acc_x;
  glo_ephem->acc[1] = gnss_glo_ephem_msg->acc_y;
  glo_ephem->acc[2] = gnss_glo_ephem_msg->acc_z;
  glo_ephem->tau_n = gnss_glo_ephem_msg->tau_n;
  glo_ephem->gamma = gnss_glo_ephem_msg->gamma;
  glo_ephem->delta_tau_n = gnss_glo_ephem_msg->delta_tau_n;
  
  return glo_ephem;
}

GnssMeasMsg meas2msg(const std::vector<ObsPtr> &meas) {
  GnssMeasMsg gnss_meas_msg;
  for (ObsPtr obs : meas) {
    GnssObsMsg obs_msg;
    uint32_t week = 0;
    double tow = time2gpst(obs->time, &week);
    obs_msg.time.week = week;
    obs_msg.time.tow = tow;
    obs_msg.sat = obs->sat;
    obs_msg.freqs = obs->freqs;
    obs_msg.CN0 = obs->CN0;
    obs_msg.LLI = obs->LLI;
    obs_msg.code = obs->code;
    obs_msg.psr = obs->psr;
    obs_msg.psr_std = obs->psr_std;
    obs_msg.adr = obs->adr;
    obs_msg.adr_std = obs->adr_std;
    obs_msg.dopp = obs->dopp;
    obs_msg.dopp_std = obs->dopp_std;
    obs_msg.status = obs->status;

    gnss_meas_msg.meas.push_back(obs_msg);
  }
  return gnss_meas_msg;
}

std::vector<ObsPtr> msg2meas(const GnssMeasMsgConstPtr &gnss_meas_msg) {
  std::vector<ObsPtr> meas;
  for (size_t i = 0; i < gnss_meas_msg->meas.size(); ++i) {
    GnssObsMsg obs_msg = gnss_meas_msg->meas[i];
    ObsPtr obs(new Obs());
    obs->time = gpst2time(obs_msg.time.week, obs_msg.time.tow);
    obs->sat = obs_msg.sat;
    obs->freqs = obs_msg.freqs;
    obs->CN0 = obs_msg.CN0;
    obs->LLI = obs_msg.LLI;
    obs->code = obs_msg.code;
    obs->psr = obs_msg.psr;
    obs->psr_std = obs_msg.psr_std;
    obs->adr = obs_msg.adr;
    obs->adr_std = obs_msg.adr_std;
    obs->dopp = obs_msg.dopp;
    obs->dopp_std = obs_msg.dopp_std;
    obs->status = obs_msg.status;

    meas.push_back(obs);
  }
  return meas;
}

} // namespace bynav_gps_driver