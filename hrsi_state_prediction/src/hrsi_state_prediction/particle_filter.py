# -*- coding: utf-8 -*-
"""
Created on Thu Aug 20 11:02:48 2015

@author: cdondrup
"""

import rospy
import xmltodict
import pybayes as pb
import numpy as np
#import time
import math
import qtc_utils as qu
from copy import deepcopy
import os
import json

DEBUG = False


class QtcInitPdf(pb.Pdf):
    def __init__(self, models, rv=None):
        self._set_rv(2, rv)
        self.models = models
        self.model = 0
        self.mu = 0

    def mean(self, cond=None):
        return self.mu

    def variance(self, cond=None):
        return 1. # No clue...

    def _get_current_state_transitions(self):
        return self.models[int(self.model)][self.mean()]

    def sample(self, cond=None):
        return self.samples(1)

    def samples(self, n, cond=None):
#        st = self._get_current_state_transitions()
        # Much quicker than repeatedly calling sample like in the default implementation
        return np.zeros((2,n)).T

    def eval_log(self, x, cond=None):
        self._check_x(x)
        st = self._get_current_state_transitions()
        if not np.all([y in st for y in x]):
            return -float('inf')
        return np.log(st[np.where(st[:,0] == x[0])][0][1])


class UniIntPdf(pb.UniPdf):
    def __init__(self, num_states, a, b, rv = None):
        super(UniIntPdf, self).__init__(a, b, rv)
        self.num_states = num_states

    def sample(self, cond = None):
        return np.round(super(UniIntPdf, self).sample(cond))

    def samples(self, n, cond = None):
        # Cheating: ensuring that every state in every model has at least one particle
        # if there are enough particles. Otherwise later models will be empty. If there
        # are not enough particles, you have much bigger problems than this anyway.
        models = [np.repeat(x,self.b[0]+1-self.a[0]) for x in np.arange(self.a[1], self.b[1]+1)]
        states = np.arange(self.a[0], self.b[0]+1)
        spam = np.array([])
        for x in models:
            spam = np.append(spam, np.append(states, x).reshape(2,-1).T)
        spam = spam.reshape(-1,2)
        ret = np.empty((n, self.shape()))
        for i in range(n):
            ret[i, :] = self.sample(cond) #spam[i, :] if i < spam.shape[0] else self.sample(cond)
        return ret


class QtcPredictionPdf(pb.CPdf):
    def __init__(self, models, rv=None, cond_rv=None):
        self._set_rvs(2, rv, 2, cond_rv)
        self.models = models
        self.mu = 0

    def mean(self, cond=None):
        return self.mu

    def _set_mean(self, cond):
        self._check_cond(cond)
        self.mu = np.round(cond[0])
        return True

    def variance(self, cond=None):
        return 0. # No clue...

    def _get_current_state_transitions(self, model):
#        print int(model), self.mean()
        return self.models[int(model)][self.mean()]

    def sample(self, cond=None):
#        print "SAMPLE PARTICLE:", cond
        return self.samples(1, cond=cond)

    def samples(self, n, cond=None):
        self._set_mean(cond)
        st = self._get_current_state_transitions(cond[1])
        # Much quicker than repeatedly calling sample like in the default implementation
#        print "TRANS:", [[int(x[0]), float(x[1])] for x in st]
        try:
            r = np.append(np.random.choice(st[:,0], p=st[:,1], size=n), np.repeat(cond[1], n)).reshape(2,-1).T
        except IndexError:
#            print cond
#            print st
            return cond
#        print "SAMPLE:", cond, "->", r
        return r

    def eval_log(self, x, cond=None):
        self._check_x(x)
        self._set_mean(cond)
        st = self._get_current_state_transitions(cond[1])
        if not round(x[0]) in st:
            return -float('inf')

        # Calculating log-likelihood for multiple draws given the probabilities of the states
#        probs = [y[1] for y in st if y[0] in np.unique(x)]
#        draws = np.bincount(x)[np.unique(x)]
#        n = len(x)
#        loglike = 0.
#        for k, p in zip(draws, probs):
#            loglike += np.log(math.factorial(n)/(math.factorial(k) * math.factorial(n-k)))+n*np.log(p)+(n-k)*np.log(1-p)
#
#        return loglike

        # Not sure if this is correct, seems too trivial. However, x can only be one element then all the rest of above function is 0.
        return np.log(st[np.where(st[:,0] == x[0])][0][1])


class QtcObservationPdf(QtcPredictionPdf):
#    __confusion_matrix = np.array([
#        [.7, .2, .1, .0],
#        [.2, .6, .2, .0],
#        [.1, .2, .7, .0],
#        [.0, .0, .0, 1.]
#    ])
    __confusion_matrix = np.array([
        [.9, .1, .0, .0],
        [.1, .8, .1, .0],
        [.0, .1, .9, .0],
        [.0, .0, .0, 1.]
    ])
#    __confusion_matrix = np.array([
#        [1., .0, .0, .0],
#        [.0, 1., .0, .0],
#        [.0, .0, .1, .0],
#        [.0, .0, .0, 1.]
#    ])

    __states = [-1, 0, 1, np.nan]

    def __init__(self, qtc_states, models, rv=None, cond_rv=None):
        super(QtcObservationPdf, self).__init__(models, rv, cond_rv)
        self.qtc_states = qtc_states

    def sample(self, cond=None):
        return self.samples(1, cond=cond)

    def samples(self, n, cond=None):
        self._set_mean(cond)
        obs = self.qtc_states[int(self.mean())]
        obs_size = 2 if np.any(np.isnan(obs)) else 4
        return np.array([
            np.random.choice(
                self.__states,
                p=self.__confusion_matrix[self.__states.index(obs[x])],
                size=n)
            for x in range(obs_size)
        ]).T

    def eval_log(self, x, cond=None):
#        print "EVAL CALLED:", x, cond
        self._check_x(x)
        self._set_mean(cond)
        obs = self.qtc_states[int(x[0])]
#        obs_size = 2 if np.any(np.isnan(obs)) else 4
        pred = self.qtc_states[int(self.mean())]
        if isinstance(pred, int):
            return -50.
#        print "HERE"
#        pred_size = 2 if np.any(np.isnan(pred)) else 4
        length_bias = 0. #if pred_size == obs_size else -15.
        multi = 1. #if pred_size == obs_size else 2.
        size = len(obs) #min(obs_size, pred_size)
#        try:
        return np.sum(np.append([
            np.log(self.__confusion_matrix[self.__states.index(pred[y])][self.__states.index(obs[y])])*multi \
                for y in np.arange(0, size) #(1, size+1, step=2) # Evaluating only on the human behaviour: [ _ ? _ ?]
        ], length_bias))
#        except TypeError:
#            print obs, pred, [pred[y] for y in range(size)]


class ParticleFilterPredictor(object):
    __qtc_type_lookup = {9: "qtcbs", 81: "qtccs", 91: "qtcbcs", 109: "qtch"}
    __latest_qtc_state = None
    __no_state__ = qu.NO_STATE
    __num_particles = 1000

    filter_bank = {}

    def __init__(self, path, qtc_type=None):
        hmm = []
        self.rules = []
        self.rule_mapping = {}
        self.model_mapping = {}
        for f in os.listdir(path):
            filename = path + '/' + f
            if f.endswith(".hmm"):
                with open(filename) as fd:
                    rospy.loginfo("Reading hmm from: %s", filename)
                    hmm.append(xmltodict.parse(fd.read()))
                    self.model_mapping[len(hmm)-1] = f.split('.')[0]
            elif f.endswith(".rules"):
                with open(filename) as fd:
                    rospy.loginfo("Reading rules from: %s", filename)
                    self.rules.append(json.load(fd))
                    self.rule_mapping[f.split('.')[0]] = len(self.rules)-1
        self.num_models = len(hmm)
        print self.model_mapping, self.rule_mapping
        self.models, self.qtc_states = self._create_model(hmm)
        self.qtc_states_no_nan = np.array(deepcopy(self.qtc_states[1:-1]))
        self.qtc_states_no_nan[np.isnan(self.qtc_states_no_nan)] = qu.NO_STATE

        if DEBUG:
            x_t = pb.RVComp(2, 'x_t')
            qpp = QtcPredictionPdf(models=self.models, rv=x_t, cond_rv=pb.RVComp(2, 'x_tp'))
            qop = QtcObservationPdf(qtc_states=self.qtc_states, models=self.models, rv=pb.RVComp(2, 'y_t'), cond_rv=[x_t])

            # prepare initial particle density:
            init_pdf = UniIntPdf((len(self.qtc_states)-1)*len(self.models), np.array([1., 0.]), np.array([90., 2.]))
            pf = ParticleFilter(500, init_pdf, qpp, qop)

            print "INIT MEAN:", pf.posterior().mean()
            p = pf.emp.particles
            print "INIT MODEL 0: %s, 1: %s" % (np.bincount(map(int,p[:,1]))[0], np.bincount(map(int,p[:,1]))[1])
            print "INIT STATES:", np.bincount(map(int,p[:,0].flatten())), len(np.bincount(map(int,p[:,0].flatten())))
            print "INIT STATES model 0:", np.bincount(map(int,p[np.where(p[:,1] == 0),0].flatten()))
            print "INIT STATES model 1:", np.bincount(map(int,p[np.where(p[:,1] == 1),0].flatten()))

            obs_sequence = [1., 18., 27., 36., 63., 90., 9.] #[1., 18., 27., 18., 27., 36.] #[1., 10., 46., 82., 9.]#, 91.]
            for idx, obs in enumerate(obs_sequence):
                print "###############################################################"
                pf.bayes(np.array([obs, np.nan]))
                print "%s OBS:" % idx, obs
                print "%s MEAN:" % idx, pf.posterior().mean()
                p = pf.emp.particles
                try:
                    print "%s MODEL 0: %s, 1: %s" % (idx, np.bincount(map(int,p[:,1]))[0], np.bincount(map(int,p[:,1]))[1])
                except:
                    pass
                print "%s STATES:" % idx, np.bincount(map(int,p[:,0].flatten()))
                try:
                    print "%s STATES model 0:" % idx, np.bincount(map(int,p[np.where(p[:,1] == 0),0].flatten()))
                except:
                    pass
                try:
                    print "%s STATES model 1:" % idx, np.bincount(map(int,p[np.where(p[:,1] == 1),0].flatten()))
                except:
                    pass
                print "%s MEDIAN: %s, STATE: %s, TRANS: %s" % (idx, np.bincount(map(int,p[:,0].flatten())).argmax(), self.qtc_states[int(np.bincount(map(int,p[:,0].flatten())).argmax())], [[int(x[0]), float(x[1])] for x in self.models[0][int(np.bincount(map(int,p[:,0].flatten())).argmax())]])

    def create_new_filter(self, uuid):
        if not uuid in self.filter_bank.keys():
            # Prepare pdfs:
            x_t = pb.RVComp(2, 'x_t')
            qpp = QtcPredictionPdf(models=self.models, rv=x_t, cond_rv=pb.RVComp(2, 'x_tp'))
            qop = QtcObservationPdf(qtc_states=self.qtc_states, models=self.models, rv=pb.RVComp(2, 'y_t'), cond_rv=[x_t])

            # prepare initial particle density:
            init_pdf = UniIntPdf((len(self.qtc_states)-1)*len(self.models), np.array([1., 0.]), np.array([float(len(self.qtc_states)-1), float(self.num_models-1)])) # Has to include 0 otherwise too many particles die because nothing transitions to 1 in passby
            self.filter_bank[uuid] = {
                "filter": ParticleFilter(self.__num_particles, init_pdf, qpp, qop),
                "last_prediction": ['?',0],
                "model_weights": np.array([0.]*self.num_models)
            }
            p = self.filter_bank[uuid]["filter"].emp.particles
            print "INIT MODEL SIZES:", np.bincount(map(int,p[:,1].flatten()), minlength=self.num_models)
            return True
        else:
            return False

    def _create_model(self, hmm):
        num_symbols = max([int(x['#text']) for x in hmm[0]['mixture']['HMM']['alphabet']['symbol']])
        qtc_states = [0]
        qtc_states.extend([x for x in qu.create_states(self.__qtc_type_lookup[num_symbols])])
        qtc_states.append(num_symbols)
        models = []
        for i, h in enumerate(hmm):
            rospy.loginfo("Loading model %s/%s" % (i+1, len(hmm)))
            model = {}
            for i in range(0,num_symbols+1):
                model[i] = np.array([np.array(map(float,[x["@target"], x['probability']])) for x in h['mixture']['HMM']['transition'] if int(x['@source']) == i and float(x['probability']) > 0.])
            model[num_symbols] = np.array([[float(num_symbols), 1.]])
            models.append(model)
        return models, qtc_states

    def predict(self, uuid, qtc, dist):
#        print qtc
#        if dist == 'und':
#            self.__latest_qtc_state = qtc[-1]
#            return ['?',0]
#        print self.__latest_qtc_state

        self.create_new_filter(uuid)
        qtc = np.array(qtc)
        print qtc
        qtc[np.isnan(qtc)] = self.__no_state__
        try:
            if self.__latest_qtc_state != None:
                new_states = qtc[np.where(np.all(qtc==self.__latest_qtc_state, axis=1))[0][-1]+1:]
            else:
                new_states = qtc
        except IndexError:
            print "Something went horribly wrong. Discarding new states."
            self.__latest_qtc_state = qtc[-1]
            new_states = []

        if len(new_states) == 0:
            new_states = [qtc[-1]]

#        print "NEW STATES:", new_states
        for state in new_states:
            state = np.array(state)
#            if len(state[state!=qu.NO_STATE]) < 4:
#                continue
            obs = np.where(np.all(self.qtc_states_no_nan==state, axis=1))[0][-1]+1
            self.filter_bank[uuid]["filter"].bayes(np.array([obs, np.nan]))
            print "###############################################################"
            print "OBS:", obs, self.qtc_states[obs]
#            print "MEAN:", self.filter_bank[uuid].posterior().mean()
            p = self.filter_bank[uuid]["filter"].emp.particles
            try:
                print "MODELS:", self.model_mapping.values()
                print "MODEL SIZES:", np.bincount(map(int,p[:,1].flatten()), minlength=self.num_models)
            except:
                pass
#            print "STATES:", np.bincount(map(int,p[:,0].flatten()))
#            try:
#                print "STATES model 0:", np.bincount(map(int,p[np.where(p[:,1] == 0),0].flatten()))
#            except:
#                pass
#            try:
#                print "STATES model 1:", np.bincount(map(int,p[np.where(p[:,1] == 1),0].flatten()))
#            except:
#                pass
#            self.filter_bank[uuid]["model_weights"][int(np.bincount(map(int,p[:,1].flatten())).argmax())] += 1.
#            model_weights = self.filter_bank[uuid]["model_weights"]/np.sum(self.filter_bank[uuid]["model_weights"])
#            best_model = np.random.choice(np.arange(self.num_models), p=model_weights, size=100)
#            best_model = np.append(best_model, np.floor(np.random.uniform(size=int(len(best_model)*.1))*3))
#            model_bins = np.bincount(map(int,best_model), minlength=self.num_models)
#            print "MODEL BINS", model_bins
#            best_model = model_bins.argmax()
#            print "MODEL WEIGHTS:", model_weights
#            print "CHOSEN MODEL:", best_model
            self.filter_bank[uuid]["last_prediction"] = np.array(self.qtc_states[np.bincount(map(int,p[:,0].flatten())).argmax()])
            print "MEDIAN: %s, STATE: %s" % (np.bincount(map(int,p[:,0].flatten())).argmax(), self.filter_bank[uuid]["last_prediction"])
#            for i in range(self.num_models):
#                print "MODEL %s BIN COUNT:" % i, np.bincount(map(int,p[:,0][p[:,1]==i].flatten()), minlength=len(self.qtc_states))
#            prediction = np.array([])
#            for particle in p:
#                par = self.filter_bank[uuid]["filter"].p_xt_xtp.sample(particle)
#                prediction = np.append(prediction, par).reshape(-1,2)
##            print prediction
#            try:
#                print "+++ PREDICTED: MEDIAN: %s, STATE: %s" % (np.bincount(map(int,prediction[:,0].flatten())).argmax(), self.qtc_states[int(np.bincount(map(int,prediction[:,0].flatten())).argmax())])#, [[int(x[0]), float(x[1])] for x in self.models[0][int(np.bincount(map(int,p[:,0].flatten())).argmax())]])
#                print "BIN COUNT:", np.bincount(map(int,prediction[:,0].flatten()))
#                s = np.array(self.qtc_states[int(np.bincount(map(int,prediction[:,0].flatten())).argmax())])
#                model_count = np.array(map(float,np.bincount(map(int,p[:,1]))))
##            print model_count
##            print model_count/self.__num_particles, np.max(model_count/self.__num_particles)
#                if np.max(model_count/self.__num_particles) > .6:
#                    self.filter_bank[uuid]["last_prediction"] = map(int,s[~np.isnan(s)].tolist())
#            except IndexError:
#                print "ERROR:", prediction

            pred = qu.nan_to_no_state(self.filter_bank[uuid]["last_prediction"])
            qtc_str = ','.join(map(str,map(int, pred)))
            rule_idx = self.rule_mapping[self.model_mapping[np.bincount(map(int,p[:,1].flatten())).argmax()]]
            states = self.rules[rule_idx][qtc_str].keys()
            probs = self.rules[rule_idx][qtc_str].values() # Both lists are always in a corresponding order
#            prediction = np.random.choice(states, p=probs)
            prediction = states[probs.index(max(probs))]
            prediction = map(int,prediction.split(','))
            print "prediction", prediction
            prediction = [prediction[0], int(pred[2]), prediction[1], int(pred[3])] \
                if prediction[1] != 9 else [prediction[0], int(pred[2])]
            print "prediction", prediction

        self.__latest_qtc_state = qtc[-1]
#        return map(int,self.filter_bank[uuid]["last_prediction"][~np.isnan(self.filter_bank[uuid]["last_prediction"])].tolist())
        return prediction


class ParticleFilter(pb.Filter):
    r"""Standard particle filter implementation with resampling.

    Specifying proposal density is currently unsupported, but planned; speak up if you want it!
    Posterior pdf is represented using :class:`~pybayes.pdfs.EmpPdf` and takes following form:

    .. math:: p(x_t|y_{1:t}) = \sum_{i=1}^n \omega_i \delta ( x_t - x_t^{(i)} )
    """

    def __init__(self, n, init_pdf, p_xt_xtp, p_yt_xt):
        r"""Initialise particle filter.

        :param int n: number of particles
        :param init_pdf: either :class:`~pybayes.pdfs.EmpPdf` instance that will be used
            directly as a posterior (and should already have initial particles sampled) or
            any other probability density which initial particles are sampled from
        :type init_pdf: :class:`~pybayes.pdfs.Pdf`
        :param p_xt_xtp: :math:`p(x_t|x_{t-1})` cpdf of state in *t* given state in *t-1*
        :type p_xt_xtp: :class:`~pybayes.pdfs.CPdf`
        :param p_yt_xt: :math:`p(y_t|x_t)` cpdf of observation in *t* given state in *t*
        :type p_yt_xt: :class:`~pybayes.pdfs.CPdf`
        """
        self.__initial_sample = True

        if not isinstance(n, int) or n < 1:
            raise TypeError("n must be a positive integer")
        if not isinstance(init_pdf, pb.Pdf):
            raise TypeError("init_pdf must be an instance ot the Pdf class")
        if not isinstance(p_xt_xtp, pb.CPdf) or not isinstance(p_yt_xt, pb.CPdf):
            raise TypeError("both p_xt_xtp and p_yt_xt must be instances of the CPdf class")

        self.init_pdf = init_pdf
        dim = init_pdf.shape()  # dimension of state
        if p_xt_xtp.shape() != dim or p_xt_xtp.cond_shape() < dim:
            raise ValueError("Expected shape() and cond_shape() of p_xt_xtp will "
                + "be (respectively greater than) {0}; ({1}, {2}) given.".format(dim,
                p_xt_xtp.shape(), p_xt_xtp.cond_shape()))
        self.p_xt_xtp = p_xt_xtp
        if p_yt_xt.cond_shape() != dim:
            raise ValueError("Expected cond_shape() of p_yt_xt will be {0}; {1} given."
                .format(dim, p_yt_xt.cond_shape()))
        self.p_yt_xt = p_yt_xt

        if isinstance(init_pdf, EmpPdf):
            self.emp = init_pdf  # use directly
        else:
            self.emp = EmpPdf(init_pdf.samples(n))

    def bayes(self, yt, cond = None):
        r"""Perform Bayes rule for new measurement :math:`y_t`; *cond* is ignored.

        :param numpy.ndarray cond: optional condition that is passed to :math:`p(x_t|x_{t-1})`
          after :math:`x_{t-1}` so that is can be rewritten as: :math:`p(x_t|x_{t-1}, c)`.

        The algorithm is as follows:

        1. generate new particles: :math:`x_t^{(i)} = \text{sample from }
           p(x_t^{(i)}|x_{t-1}^{(i)}) \quad \forall i`
        2. recompute weights: :math:`\omega_i = p(y_t|x_t^{(i)})
           \omega_i \quad \forall i`
        3. normalise weights
        4. resample particles
        """
        bla = []
        bla0 = []
        bla1 = []
        loglike = []
        for i in range(self.emp.particles.shape[0]):
            # generate new ith particle:
            if not self.__initial_sample: # Only sample if not the first round
                self.emp.particles[i] = self.p_xt_xtp.sample(self.emp.particles[i])

            # recompute ith weight:
            self.emp.weights[i] *= math.exp(self.p_yt_xt.eval_log(yt, self.emp.particles[i]))
            loglike.append(self.p_yt_xt.eval_log(yt, self.emp.particles[i]))

            if DEBUG:
                bla.append(np.append(self.emp.particles[i], self.emp.weights[i]))
                if self.emp.particles[i][1] == 0: bla0.append(self.emp.particles[i][0])
                else: bla1.append(self.emp.particles[i][0])

#        print "LIKELIHOODS:", max(loglike), min(loglike), np.mean(loglike)
        if DEBUG:
            bla = np.array(bla)
            b = np.ascontiguousarray(bla).view(np.dtype((np.void, bla.dtype.itemsize * bla.shape[1])))
            _, idx = np.unique(b, return_index=True)

            print np.array([[int(x[0]), int(x[1]), int(x[2]*1000000.)] for x in bla[idx]], dtype=int)
            print "PARTICLE 0", np.bincount(bla0)
            print "PARTICLE 1", np.bincount(bla1)

        # assure that weights are normalised
        self.emp.normalise_weights()
        # resample
        self.emp.resample(self.init_pdf)

        self.__initial_sample = False
        return True

    def posterior(self):
        return self.emp


class EmpPdf(pb.EmpPdf):
    r"""An abstraction of empirical probability density functions that provides common methods such
    as weight normalisation. Extends :class:`Pdf`.

    :var numpy.ndarray weights: 1D array of particle weights
       :math:`\omega_i >= 0 \forall i; \quad \sum \omega_i = 1`
    """

    __starvation_factor = 0.9

    def get_resample_indices(self):
        r"""Calculate first step of resampling process (dropping low-weight particles and
        replacing them with more weighted ones.

        :return: integer array of length n: :math:`(a_1, a_2 \dots a_n)` where
            :math:`a_i` means that particle at ith place should be replaced with particle
            number :math:`a_i`
        :rtype: :class:`numpy.ndarray` of ints

        *This method doesnt modify underlying pdf in any way - it merely calculates how
        particles should be replaced.*
        """
        n = self.weights.shape[0]
        cum_weights = np.cumsum(self.weights)

        m = int(np.round(float(n)*self.__starvation_factor))
        u = np.empty(m)
        fuzz = np.random.uniform()
        for i in range(m):
            u[i] = (i + fuzz) / m

        # calculate number of babies for each particle
        baby_indices = np.empty(n, dtype=int)  # index array: a[i] contains index of
        # original particle that should be at i-th place in new particle array
        j = 0
        for i in range(m):
            while u[i] > cum_weights[j]:
                j += 1
            baby_indices[i] = j

        for i in np.arange(m, n):
            baby_indices[i] = -1
        return baby_indices

    def resample(self, init_pdf):
        super(EmpPdf, self).resample()
        n = self.weights.shape[0]
        m = int(np.round(float(n)*self.__starvation_factor))
        for i in np.arange(m, n):
            self.particles[i] = init_pdf.sample()
